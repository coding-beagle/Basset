//! A finished picture: eight-bit sRGB pixels, and PNG out.

use std::io::Write;
use std::path::Path;

#[derive(Debug, thiserror::Error)]
pub enum ImageError {
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),
    #[error("png encoding failed: {0}")]
    Png(#[from] png::EncodingError),
}

/// RGBA, eight bits a channel, sRGB, rows top to bottom. Alpha is always opaque: a render
/// has a background.
#[derive(Clone, Debug, PartialEq)]
pub struct Image {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<u8>,
    /// Samples per pixel the picture was averaged over.
    pub samples: u32,
}

impl Image {
    pub fn pixel(&self, x: u32, y: u32) -> [u8; 4] {
        let i = ((y * self.width + x) * 4) as usize;
        [
            self.pixels[i],
            self.pixels[i + 1],
            self.pixels[i + 2],
            self.pixels[i + 3],
        ]
    }

    pub fn write_png<W: Write>(&self, w: W) -> Result<(), ImageError> {
        let mut encoder = png::Encoder::new(w, self.width, self.height);
        encoder.set_color(png::ColorType::Rgba);
        encoder.set_depth(png::BitDepth::Eight);
        encoder.set_source_srgb(png::SrgbRenderingIntent::Perceptual);
        let mut writer = encoder.write_header()?;
        writer.write_image_data(&self.pixels)?;
        writer.finish()?;
        Ok(())
    }

    /// Writes through a temporary file and a rename, as documents are saved, so a failed
    /// write never leaves half a picture where a whole one was.
    pub fn save_png(&self, path: impl AsRef<Path>) -> Result<(), ImageError> {
        let path = path.as_ref();
        let tmp = path.with_extension("png.tmp");
        {
            let file = std::io::BufWriter::new(std::fs::File::create(&tmp)?);
            self.write_png(file)?;
        }
        std::fs::rename(&tmp, path)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_png_starts_with_its_signature_and_carries_the_size() {
        let image = Image {
            width: 3,
            height: 2,
            pixels: vec![128; 3 * 2 * 4],
            samples: 1,
        };
        let mut bytes = Vec::new();
        image.write_png(&mut bytes).unwrap();
        assert_eq!(&bytes[..8], b"\x89PNG\r\n\x1a\n");
        let decoder = png::Decoder::new(std::io::Cursor::new(bytes));
        let reader = decoder.read_info().unwrap();
        assert_eq!(reader.info().width, 3);
        assert_eq!(reader.info().height, 2);
    }
}
