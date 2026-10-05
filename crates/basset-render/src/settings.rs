//! A scene's settings — Fusion's Scene Settings dialog — saved with the document.

use serde::{Deserialize, Serialize};

use crate::color::Srgb;
use crate::environment::{EnvironmentKind, Lighting};

/// What is behind the model where nothing else is.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Background {
    /// The environment's own sky, without its lights: a soft box seen directly is a white
    /// blot, and nobody photographs the lamp.
    #[default]
    Environment,
    /// One flat colour, as for a catalogue cut-out.
    Solid(Srgb),
}

/// Every field defaults to what Fusion opens a new design's render with, so a document
/// saved before scenes were saved opens in the Photo Booth.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SceneSettings {
    pub environment: EnvironmentKind,
    /// Exposure in stops: each step doubles or halves every light.
    pub brightness: f32,
    /// Turn of the environment about the vertical, degrees.
    pub rotation: f32,
    pub background: Background,
    /// A floor under the model, at the bottom of its bounding box, that catches its
    /// shadow and fades into the background with distance.
    pub ground_plane: bool,
    /// Whether the floor is glossy enough to reflect the model.
    pub ground_reflections: bool,
    /// How blurred those reflections are.
    pub ground_roughness: f32,
    /// Depth of field: everything at the camera's target in focus, nearer and further
    /// blurred by how far open the aperture is.
    pub depth_of_field: bool,
    /// Lens aperture radius as a fraction of the distance to the focus point, so the blur
    /// looks the same whether the part is a washer or an engine block.
    pub aperture: f32,
}

impl Default for SceneSettings {
    fn default() -> Self {
        Self {
            environment: EnvironmentKind::default(),
            brightness: 0.0,
            rotation: 0.0,
            background: Background::default(),
            ground_plane: true,
            ground_reflections: false,
            ground_roughness: 0.15,
            depth_of_field: false,
            aperture: 0.02,
        }
    }
}

impl SceneSettings {
    /// The multiplier the brightness stands for.
    pub fn exposure(&self) -> f32 {
        2f32.powf(self.brightness.clamp(-10.0, 10.0))
    }

    /// The environment with brightness and rotation applied.
    pub fn lighting(&self) -> Lighting {
        Lighting::new(
            &self.environment.describe(),
            self.exposure(),
            f64::from(self.rotation).to_radians(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_empty_object_is_the_default_scene() {
        let s: SceneSettings = serde_json::from_str("{}").unwrap();
        assert_eq!(s, SceneSettings::default());
    }

    #[test]
    fn settings_round_trip_through_json() {
        let s = SceneSettings {
            environment: EnvironmentKind::Dusk,
            background: Background::Solid(Srgb::hex(0x102030)),
            ground_reflections: true,
            ..SceneSettings::default()
        };
        let json = serde_json::to_string(&s).unwrap();
        assert_eq!(serde_json::from_str::<SceneSettings>(&json).unwrap(), s);
    }

    #[test]
    fn a_stop_doubles_the_light() {
        let s = SceneSettings {
            brightness: 1.0,
            ..SceneSettings::default()
        };
        assert!((s.exposure() - 2.0).abs() < 1e-6);
        let base = SceneSettings::default().lighting();
        let brighter = s.lighting();
        assert!((brighter.zenith[0] - 2.0 * base.zenith[0]).abs() < 1e-6);
    }
}
