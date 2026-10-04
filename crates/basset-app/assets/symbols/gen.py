#!/usr/bin/env python3
"""Turn symbols.json (one SVG per tool) into src/editor/symbol_data.rs.

    python3 crates/basset-app/assets/symbols/gen.py

The symbols are designed as SVG, at 24x24, because that is where they can be drawn,
previewed and compared. The app paints them with egui's painter rather than rasterising
SVG, so they stay sharp at every size and take the button's colour. This script does the
translation once, offline: every element becomes polylines in the 24-unit frame, with
curves flattened and concave fills triangulated. The app then only has to scale and paint.

Only what the set uses is supported: line, polyline, polygon, circle, ellipse, rect and
path (M L H V Q C A Z, absolute or relative), with fill/stroke of currentColor,
var(--accent) or var(--faint), plus opacity, stroke-width and stroke-dasharray.
Anything else is an error, so a new symbol cannot silently lose a part.
"""
import json
import math
import os
import re
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))

# Symbol name in the JSON -> Rust variant, in the order they are generated.
VARIANTS = {
    "Create Sketch": "CreateSketch", "Extrude": "Extrude", "Revolve": "Revolve",
    "Sweep": "Sweep", "Loft": "Loft", "Fillet": "Fillet", "Chamfer": "Chamfer",
    "Thread": "Thread", "Combine": "Combine", "Move": "Move",
    "Offset Plane": "OffsetPlane", "Plane at Angle": "AngledPlane",
    "New Component": "Component", "Simulate": "Simulate", "Measure": "Measure",
    "Fit": "Fit", "Select": "Select", "Line": "Line", "Rectangle": "Rectangle",
    "Center Rectangle": "CenterRectangle", "Circle": "Circle",
    "2-Point Circle": "Circle2Point", "3-Point Circle": "Circle3Point",
    "3-Point Arc": "Arc3Point", "Center Arc": "ArcCenter", "Polygon": "Polygon",
    "Slot": "Slot", "Overall Slot": "SlotOverall",
    "Center Point Slot": "SlotCenterPoint", "Text": "Text",
    "Dimension": "Dimension", "Trim": "Trim", "Break": "Break",
    "Sketch Fillet": "SketchFillet", "Construction": "Construction",
    "Delete": "Delete", "Sketch Move": "SketchMove", "Pattern": "Pattern",
    "Offset": "Offset", "Finish Sketch": "FinishSketch",
    "Coincident": "Coincident", "Horizontal": "Horizontal", "Vertical": "Vertical",
    "Parallel": "Parallel", "Perpendicular": "Perpendicular", "Tangent": "Tangent",
    "Equal": "Equal", "Concentric": "Concentric", "Midpoint": "Midpoint",
    "Symmetric": "Symmetric", "Fix": "Fix",
}

INKS = {"currentColor": "Main", "var(--accent)": "Accent", "var(--faint)": "Faint",
        "black": "Main"}
INHERITED = {"fill", "stroke", "stroke-width", "stroke-dasharray", "stroke-linecap",
             "stroke-linejoin", "fill-rule"}
CURVE_STEPS = 10


def attrs(el, root):
    a = {k: v for k, v in root.attrib.items() if k in INHERITED}
    a.update(el.attrib)
    for decl in el.attrib.get("style", "").split(";"):
        if ":" in decl:
            k, v = decl.split(":", 1)
            a[k.strip()] = v.strip()
    return a


def num(a, k, d=0.0):
    return float(a.get(k, d))


def ring(cx, cy, rx, ry):
    n = 24 if max(rx, ry) > 2.5 else 16
    return [(cx + rx * math.cos(2 * math.pi * i / n), cy + ry * math.sin(2 * math.pi * i / n))
            for i in range(n)]


def arc_points(p0, rx, ry, phi, large, sweep, p1):
    """SVG endpoint arc to points (excluding p0), per the SVG implementation notes."""
    if rx == 0 or ry == 0:
        return [p1]
    phi = math.radians(phi)
    cp, sp = math.cos(phi), math.sin(phi)
    dx, dy = (p0[0] - p1[0]) / 2, (p0[1] - p1[1]) / 2
    x1, y1 = cp * dx + sp * dy, -sp * dx + cp * dy
    rx, ry = abs(rx), abs(ry)
    lam = x1 * x1 / (rx * rx) + y1 * y1 / (ry * ry)
    if lam > 1:
        rx, ry = rx * math.sqrt(lam), ry * math.sqrt(lam)
    num_ = rx * rx * ry * ry - rx * rx * y1 * y1 - ry * ry * x1 * x1
    den = rx * rx * y1 * y1 + ry * ry * x1 * x1
    co = math.sqrt(max(0.0, num_ / den)) if den else 0.0
    if large == sweep:
        co = -co
    cx1, cy1 = co * rx * y1 / ry, -co * ry * x1 / rx
    cx = cp * cx1 - sp * cy1 + (p0[0] + p1[0]) / 2
    cy = sp * cx1 + cp * cy1 + (p0[1] + p1[1]) / 2

    def ang(ux, uy, vx, vy):
        a = math.atan2(ux * vy - uy * vx, ux * vx + uy * vy)
        return a

    t1 = ang(1, 0, (x1 - cx1) / rx, (y1 - cy1) / ry)
    dt = ang((x1 - cx1) / rx, (y1 - cy1) / ry, (-x1 - cx1) / rx, (-y1 - cy1) / ry)
    if not sweep and dt > 0:
        dt -= 2 * math.pi
    elif sweep and dt < 0:
        dt += 2 * math.pi
    n = max(4, int(abs(dt) / (math.pi / 12)))
    pts = []
    for i in range(1, n + 1):
        t = t1 + dt * i / n
        x, y = rx * math.cos(t), ry * math.sin(t)
        pts.append((cp * x - sp * y + cx, sp * x + cp * y + cy))
    pts[-1] = p1
    return pts


def path_subpaths(d):
    toks = re.findall(r"[A-Za-z]|-?(?:\d+\.?\d*|\.\d+)(?:e-?\d+)?", d)
    subs, cur, pos, start, i, cmd = [], None, (0.0, 0.0), (0.0, 0.0), 0, None

    def take(n):
        nonlocal i
        vals = [float(t) for t in toks[i:i + n]]
        i += n
        return vals

    while i < len(toks):
        if re.match(r"[A-Za-z]", toks[i]):
            cmd = toks[i]
            i += 1
        rel = cmd.islower()
        c = cmd.upper()
        ox, oy = pos if rel else (0.0, 0.0)
        if c == "M":
            x, y = take(2)
            pos = start = (ox + x, oy + y)
            cur = [pos]
            subs.append([cur, False])
            cmd = "l" if rel else "L"
        elif c == "L":
            x, y = take(2)
            pos = (ox + x, oy + y)
            cur.append(pos)
        elif c == "H":
            (x,) = take(1)
            pos = ((pos[0] if rel else 0) + x, pos[1])
            cur.append(pos)
        elif c == "V":
            (y,) = take(1)
            pos = (pos[0], (pos[1] if rel else 0) + y)
            cur.append(pos)
        elif c == "Q":
            qx, qy, x, y = take(4)
            q, e = (ox + qx, oy + qy), (ox + x, oy + y)
            for k in range(1, CURVE_STEPS + 1):
                t = k / CURVE_STEPS
                cur.append(((1 - t) ** 2 * pos[0] + 2 * (1 - t) * t * q[0] + t * t * e[0],
                            (1 - t) ** 2 * pos[1] + 2 * (1 - t) * t * q[1] + t * t * e[1]))
            pos = e
        elif c == "C":
            ax, ay, bx, by, x, y = take(6)
            a_, b_, e = (ox + ax, oy + ay), (ox + bx, oy + by), (ox + x, oy + y)
            for k in range(1, CURVE_STEPS + 1):
                t = k / CURVE_STEPS
                u = 1 - t
                cur.append(tuple(u ** 3 * p0 + 3 * u * u * t * p1 + 3 * u * t * t * p2 + t ** 3 * p3
                                 for p0, p1, p2, p3 in zip(pos, a_, b_, e)))
            pos = e
        elif c == "A":
            rx, ry, phi, large, sweep, x, y = take(7)
            e = (ox + x, oy + y)
            cur.extend(arc_points(pos, rx, ry, phi, int(large), int(sweep), e))
            pos = e
        elif c == "Z":
            subs[-1][1] = True
            pos = start
        else:
            raise SystemExit(f"unsupported path command {cmd!r} in {d!r}")
    return [(pts, closed) for pts, closed in subs if len(pts) > 1]


def geometry(el, a):
    tag = el.tag.split("}")[-1]
    if tag == "line":
        return [([(num(a, "x1"), num(a, "y1")), (num(a, "x2"), num(a, "y2"))], False)]
    if tag in ("polyline", "polygon"):
        v = [float(t) for t in re.findall(r"-?(?:\d+\.?\d*|\.\d+)", a["points"])]
        return [(list(zip(v[0::2], v[1::2])), tag == "polygon")]
    if tag == "circle":
        return [(ring(num(a, "cx"), num(a, "cy"), num(a, "r"), num(a, "r")), True)]
    if tag == "ellipse":
        return [(ring(num(a, "cx"), num(a, "cy"), num(a, "rx"), num(a, "ry")), True)]
    if tag == "rect":
        x, y, w, h = num(a, "x"), num(a, "y"), num(a, "width"), num(a, "height")
        r = min(num(a, "rx", a.get("ry", 0)), w / 2, h / 2)
        if r <= 0:
            return [([(x, y), (x + w, y), (x + w, y + h), (x, y + h)], True)]
        pts = []
        for cx, cy, a0 in [(x + w - r, y + r, -90), (x + w - r, y + h - r, 0),
                           (x + r, y + h - r, 90), (x + r, y + r, 180)]:
            for k in range(5):
                t = math.radians(a0 + 90 * k / 4)
                pts.append((cx + r * math.cos(t), cy + r * math.sin(t)))
        return [(pts, True)]
    if tag == "path":
        return path_subpaths(a["d"])
    raise SystemExit(f"unsupported element <{tag}>")


def area2(pts):
    return sum(pts[i][0] * pts[i - 1][1] - pts[i - 1][0] * pts[i][1] for i in range(len(pts)))


def dedupe(pts):
    out = []
    for p in pts:
        if not out or math.dist(p, out[-1]) > 1e-4:
            out.append(p)
    if len(out) > 2 and math.dist(out[0], out[-1]) < 1e-4:
        out.pop()
    return out


def convex(pts):
    sign = 0
    n = len(pts)
    for i in range(n):
        (ax, ay), (bx, by), (cx, cy) = pts[i], pts[(i + 1) % n], pts[(i + 2) % n]
        z = (bx - ax) * (cy - by) - (by - ay) * (cx - bx)
        if abs(z) < 1e-6:
            continue
        s = 1 if z > 0 else -1
        if sign and s != sign:
            return False
        sign = s
    return True


def triangulate(pts):
    """Ear clipping; the outlines are small and simple, so this is plenty."""
    idx = list(range(len(pts)))
    if area2(pts) > 0:  # area2 is the negated shoelace: make the winding positive
        idx.reverse()
    tris = []

    def inside(p, a, b, c):
        def s(p1, p2, p3):
            return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])
        d1, d2, d3 = s(p, a, b), s(p, b, c), s(p, c, a)
        e = 1e-9
        return (d1 > e and d2 > e and d3 > e) or (d1 < -e and d2 < -e and d3 < -e)

    guard = 0
    while len(idx) > 3 and guard < 10000:
        guard += 1
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = pts[i0], pts[i1], pts[i2]
            cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
            if abs(cross) < 1e-9:
                idx.pop(k)  # a straight-through vertex encloses nothing
                break
            if cross < 0:
                continue
            if any(inside(pts[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            break
        else:
            raise SystemExit("could not triangulate a fill")
    tris.append(tuple(idx))
    return tris


def ink(v):
    if v in (None, "none"):
        return None
    if v not in INKS:
        raise SystemExit(f"unsupported colour {v!r}")
    return INKS[v]


def f(x):
    s = f"{x:.2f}".rstrip("0").rstrip(".")
    return s if "." in s else s + ".0"


def prims(svg):
    root = ET.fromstring(svg)
    out = []
    for el in root.iter():
        if el is root:
            continue
        a = attrs(el, root)
        tag = el.tag.split("}")[-1]
        fill = ink(a.get("fill", "black"))
        stroke = ink(a.get("stroke"))
        if a.get("fill-rule") == "evenodd" and fill:
            raise SystemExit("evenodd fills are not supported")
        if tag in ("line", "polyline"):
            fill = None if tag == "line" else fill
        opacity = float(a.get("opacity", 1))
        paths = []
        for pts, closed in geometry(el, a):
            pts = dedupe(pts)
            tris = []
            if fill and len(pts) > 2 and not convex(pts):
                tris = triangulate(pts)
            paths.append((pts, closed, tris))
        width = float(a.get("stroke-width", 1))
        dash = a.get("stroke-dasharray")
        if dash and dash != "none":
            d = [float(t) for t in re.split(r"[\s,]+", dash.strip())]
            if len(d) == 1:
                d = d * 2
            if len(d) != 2:
                raise SystemExit(f"only two-value dashes are supported, got {dash!r}")
            dash = d
        else:
            dash = None
        out.append(dict(
            fill=fill and (fill, opacity * float(a.get("fill-opacity", 1))),
            stroke=stroke and (stroke, opacity * float(a.get("stroke-opacity", 1)), width, dash),
            paths=paths))
    return out


def rust(p):
    paths = []
    for pts, closed, tris in p["paths"]:
        ps = ", ".join(f"[{f(x)}, {f(y)}]" for x, y in pts)
        ts = ", ".join(f"[{a}, {b}, {c}]" for a, b, c in tris)
        paths.append(f"Path {{ pts: &[{ps}], closed: {str(closed).lower()}, tris: &[{ts}] }}")
    fill = "None"
    if p["fill"]:
        fill = f"Some(Paint {{ ink: Ink::{p['fill'][0]}, alpha: {f(p['fill'][1])} }})"
    stroke = "None"
    if p["stroke"]:
        k, alpha, width, dash = p["stroke"]
        d = f"Some([{f(dash[0])}, {f(dash[1])}])" if dash else "None"
        stroke = (f"Some(Pen {{ ink: Ink::{k}, alpha: {f(alpha)}, width: {f(width)}, "
                  f"dash: {d} }})")
    return f"Prim {{ fill: {fill}, stroke: {stroke}, paths: &[{', '.join(paths)}] }}"


def main():
    src = json.load(open(os.path.join(HERE, "symbols.json")))
    icons = src["icons"]
    missing = [n for n in VARIANTS if n not in icons]
    extra = [n for n in icons if n not in VARIANTS]
    if missing or extra:
        raise SystemExit(f"missing {missing}, unknown {extra}")
    lines = [
        "// @generated by assets/symbols/gen.py from assets/symbols/symbols.json.",
        "// Edit the SVG there and rerun the script; do not edit this file by hand.",
        "",
        "use super::symbols::{Ink, Paint, Path, Pen, Prim, Symbol};",
        "",
        "#[rustfmt::skip]",
        "pub(super) fn prims(symbol: Symbol) -> &'static [Prim] {",
        "    match symbol {",
    ]
    for name, var in VARIANTS.items():
        try:
            body = ",\n            ".join(rust(p) for p in prims(icons[name]))
        except SystemExit as e:
            raise SystemExit(f"{name}: {e}")
        lines.append(f"        // {name}")
        lines.append(f"        Symbol::{var} => &[\n            {body},\n        ],")
    lines += ["    }", "}", ""]
    out = os.path.join(HERE, "..", "..", "src", "editor", "symbol_data.rs")
    open(out, "w").write("\n".join(lines))
    print("wrote", os.path.normpath(out), "with", len(VARIANTS), "symbols")


if __name__ == "__main__":
    main()
