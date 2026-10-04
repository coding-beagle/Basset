//! Colour ramps for painting a scalar over a mesh.

/// A value in `[0, 1]` as a colour from blue through cyan, green and yellow to red: the
/// ramp every stress plot since the first one has used, so a reader needs no legend to
/// know which end is the hot one. Linear RGB, as the renderer takes it. Out-of-range
/// input is clamped, and NaN reads as the cold end rather than as a hole in the plot.
pub fn stress_ramp(t: f32) -> [f32; 3] {
    let t = if t.is_nan() { 0.0 } else { t.clamp(0.0, 1.0) };
    // Four linear pieces between five anchors.
    let anchors: [[f32; 3]; 5] = [
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 1.0],
        [0.0, 1.0, 0.0],
        [1.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
    ];
    let x = t * 4.0;
    let i = (x.floor() as usize).min(3);
    let f = x - i as f32;
    let (a, b) = (anchors[i], anchors[i + 1]);
    [
        a[0] + (b[0] - a[0]) * f,
        a[1] + (b[1] - a[1]) * f,
        a[2] + (b[2] - a[2]) * f,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_ends_are_blue_and_red_and_the_middle_green() {
        assert_eq!(stress_ramp(0.0), [0.0, 0.0, 1.0]);
        assert_eq!(stress_ramp(1.0), [1.0, 0.0, 0.0]);
        assert_eq!(stress_ramp(0.5), [0.0, 1.0, 0.0]);
        assert_eq!(stress_ramp(-3.0), stress_ramp(0.0));
        assert_eq!(stress_ramp(7.0), stress_ramp(1.0));
        assert_eq!(stress_ramp(f32::NAN), stress_ramp(0.0));
    }
}
