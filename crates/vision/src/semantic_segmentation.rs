use color_eyre::Result;
use context_attribute::context;
use framework::{AdditionalOutput, MainOutput};
use itertools::Itertools;
use types::{ycbcr422_image::YCbCr422Image, Rgb, YCbCr422};

pub struct ImageSegmenter {
    scratchpad: Vec<Rgb>,
}

#[context]
pub struct CreationContext {}

#[context]
pub struct CycleContext {
    image: Input<YCbCr422Image, "image">,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    segmented_image: MainOutput<Vec<Rgb>>,
}

impl ImageSegmenter {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self {
            scratchpad: Vec::new(),
        })
    }

    pub fn cycle(&mut self, mut context: CycleContext) -> Result<MainOutputs> {
        let image = context.image;
        self.downsample_image_into_rgb::<4>(image);

        Ok(MainOutputs {
            segmented_image: self.scratchpad.into(),
        })
    }

    fn downsample_image_into_rgb<const DOWNSAMPLE_RATIO: usize>(&mut self, image: &YCbCr422Image) {
        let height = image.height() as usize;
        let width = image.width() as usize;

        assert!(
            height % DOWNSAMPLE_RATIO == 0,
            "the image height {} is not divisible by the downsample ratio {}",
            height,
            DOWNSAMPLE_RATIO
        );
        assert!(
            width % DOWNSAMPLE_RATIO == 0,
            "the image width {} is not divisible by the downsample ratio {}",
            width,
            DOWNSAMPLE_RATIO
        );

        self.scratchpad.clear();
        let image_buffer = image
            .buffer()
            .iter()
            .chunks(width)
            .step_by(DOWNSAMPLE_RATIO)
            .map(|row| row.step_by(DOWNSAMPLE_RATIO));

        for row in image.buffer().chunks(width).step_by(DOWNSAMPLE_RATIO) {
            for pixel in row.iter().step_by(DOWNSAMPLE_RATIO) {
                let pixel_rgb = Rgb::from(*pixel);
                self.scratchpad.push(pixel_rgb);
            }
        }
    }
}

#[cfg(test)]
mod tests {}
