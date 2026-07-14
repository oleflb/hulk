use color_eyre::{
    Result,
    eyre::{bail, eyre},
};
use eframe::egui::{self, ColorImage, TextureHandle, TextureOptions, Vec2};
use image::RgbImage;
use opencv::objdetect;
use ros2::sensor_msgs::image::Image;

use crate::marker::{DetectedTag, detect_markers};

pub(crate) struct LoadedImage {
    pub(crate) texture: TextureHandle,
    pub(crate) size: Vec2,
    pub(crate) tags: Vec<DetectedTag>,
}

pub(crate) fn load_image(
    context: &egui::Context,
    detector: &objdetect::ArucoDetector,
    id: &'static str,
    image: &Image,
) -> Result<LoadedImage> {
    if image.width == 0 || image.height == 0 {
        bail!("image has no pixels: {}x{}", image.width, image.height);
    }

    let rgb_image: RgbImage = image
        .clone()
        .try_into()
        .map_err(|error: image::ImageError| eyre!(error))?;
    let tags = detect_markers(detector, &rgb_image)?;
    let texture = context.load_texture(
        id,
        ColorImage::from_rgb(
            [rgb_image.width() as usize, rgb_image.height() as usize],
            rgb_image.as_raw(),
        ),
        TextureOptions::LINEAR,
    );

    Ok(LoadedImage {
        texture,
        size: Vec2::new(rgb_image.width() as f32, rgb_image.height() as f32),
        tags,
    })
}
