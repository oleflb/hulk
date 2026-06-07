use std::path::Path;

use color_eyre::{
    Result,
    eyre::{ContextCompat, bail, ensure},
};

use ort::{
    execution_providers::TensorRTExecutionProvider,
    inputs,
    session::{Session, SessionOutputs, builder::GraphOptimizationLevel},
    value::TensorRef,
};
use ros2::sensor_msgs::image::Image;
use types::stereo_image_pair::StereoImagePair;

pub const KEYPOINTS: usize = 512;

pub struct FeatureExtractor {
    session: Session,
}

pub struct FeatureOutput<'a> {
    outputs: SessionOutputs<'a>,
}

pub struct FrameFeatures<'a> {
    keypoints: &'a [f32],
    valid: &'a [bool],
}

pub struct Matches<'a> {
    matches: &'a [i32],
    scores: &'a [f32],
}

impl FeatureExtractor {
    pub fn new(path: impl AsRef<Path>) -> Result<Self> {
        let parent = path.as_ref().parent().wrap_err("failed to find parent")?;
        let tensorrt = TensorRTExecutionProvider::default()
            .with_device_id(0)
            .with_fp16(true)
            .with_engine_cache(true)
            .with_engine_cache_path(parent.display())
            .build();

        let session = Session::builder()?
            .with_execution_providers([tensorrt])?
            .with_optimization_level(GraphOptimizationLevel::Level3)?
            .with_intra_threads(2)?
            .commit_from_file(path)?;

        Ok(Self { session })
    }

    pub fn extract<'a>(
        &'a mut self,
        previous: &StereoImagePair,
        current: &StereoImagePair,
    ) -> Result<FeatureOutput<'a>> {
        check_stereo_pair_support(previous)?;
        check_stereo_pair_support(current)?;
        ensure_same_shape(
            &previous.left,
            &current.left,
            "previous left",
            "current left",
        )?;

        let previous_left = image_tensor(&previous.left)?;
        let previous_right = image_tensor(&previous.right)?;
        let current_left = image_tensor(&current.left)?;
        let current_right = image_tensor(&current.right)?;

        let outputs = self.session.run(inputs![
            "previous_left" => previous_left,
            "previous_right" => previous_right,
            "current_left" => current_left,
            "current_right" => current_right,
        ])?;

        Ok(FeatureOutput { outputs })
    }
}

impl<'a> FeatureOutput<'a> {
    pub fn current_left(&self) -> Result<FrameFeatures<'_>> {
        self.frame("current_left_keypoints", "current_left_valid")
    }

    pub fn current_right(&self) -> Result<FrameFeatures<'_>> {
        self.frame("current_right_keypoints", "current_right_valid")
    }

    pub fn stereo_matches(&self) -> Result<Matches<'_>> {
        self.matches("stereo_matches", "stereo_scores")
    }

    pub fn temporal_matches(&self) -> Result<Matches<'_>> {
        self.matches("temporal_matches", "temporal_scores")
    }

    fn frame(&self, keypoints_name: &str, valid_name: &str) -> Result<FrameFeatures<'_>> {
        let keypoints = self.tensor_f32(keypoints_name)?;
        let valid = self.tensor_bool(valid_name)?;

        ensure!(
            keypoints.len() == KEYPOINTS * 2,
            "unexpected {keypoints_name} length: {}",
            keypoints.len()
        );
        ensure!(
            valid.len() == KEYPOINTS,
            "unexpected {valid_name} length: {}",
            valid.len()
        );

        Ok(FrameFeatures { keypoints, valid })
    }

    fn matches(&self, matches_name: &str, scores_name: &str) -> Result<Matches<'_>> {
        let matches = self.tensor_i32(matches_name)?;
        let scores = self.tensor_f32(scores_name)?;

        ensure!(
            matches.len() == KEYPOINTS,
            "unexpected {matches_name} length: {}",
            matches.len()
        );
        ensure!(
            scores.len() == KEYPOINTS,
            "unexpected {scores_name} length: {}",
            scores.len()
        );

        Ok(Matches { matches, scores })
    }

    fn tensor_f32(&self, name: &str) -> Result<&[f32]> {
        let output = self
            .outputs
            .get(name)
            .wrap_err_with(|| format!("missing model output '{name}'"))?;
        let (_, data) = output.try_extract_tensor::<f32>()?;
        Ok(data)
    }

    fn tensor_i32(&self, name: &str) -> Result<&[i32]> {
        let output = self
            .outputs
            .get(name)
            .wrap_err_with(|| format!("missing model output '{name}'"))?;
        let (_, data) = output.try_extract_tensor::<i32>()?;
        Ok(data)
    }

    fn tensor_bool(&self, name: &str) -> Result<&[bool]> {
        let output = self
            .outputs
            .get(name)
            .wrap_err_with(|| format!("missing model output '{name}'"))?;
        let (_, data) = output.try_extract_tensor::<bool>()?;
        Ok(data)
    }
}

impl FrameFeatures<'_> {
    pub fn keypoint(&self, index: usize) -> Option<[f32; 2]> {
        let offset = index.checked_mul(2)?;
        Some([
            *self.keypoints.get(offset)?,
            *self.keypoints.get(offset + 1)?,
        ])
    }

    pub fn is_valid(&self, index: usize) -> bool {
        self.valid.get(index).copied().unwrap_or(false)
    }
}

impl Matches<'_> {
    pub fn left_to_right(&self) -> impl Iterator<Item = (usize, usize, f32)> + '_ {
        self.matches
            .iter()
            .zip(self.scores.iter())
            .enumerate()
            .filter_map(|(left_index, (&right_index, &score))| {
                let right_index = usize::try_from(right_index).ok()?;
                (score > 0.0 && right_index < KEYPOINTS).then_some((left_index, right_index, score))
            })
    }
}

fn image_tensor(image: &Image) -> Result<TensorRef<'_, u8>> {
    TensorRef::from_array_view((
        [image.height as usize / 2, image.width as usize / 2, 6],
        image.data.as_ref(),
    ))
    .map_err(Into::into)
}

fn check_stereo_pair_support(stereo: &StereoImagePair) -> Result<()> {
    check_image_support(&stereo.left)?;
    check_image_support(&stereo.right)?;
    ensure_same_shape(&stereo.left, &stereo.right, "left", "right")
}

fn check_image_support(image: &Image) -> Result<()> {
    if image.encoding != "nv12" {
        bail!("unsupported encoding: {}", image.encoding);
    }

    let height = image.height as usize;
    let width = image.width as usize;

    if !(width.is_multiple_of(32) && height.is_multiple_of(32)) {
        bail!(
            "image dimensions must be multiples of 32: {}x{}",
            width,
            height
        );
    }

    if image.data.len() != height * width * 3 / 2 {
        bail!("image data length does not match dimensions");
    }

    Ok(())
}

fn ensure_same_shape(left: &Image, right: &Image, left_name: &str, right_name: &str) -> Result<()> {
    if left.height != right.height
        || left.width != right.width
        || left.data.len() != right.data.len()
    {
        bail!(
            "{left_name} and {right_name} images must have the same shape: {}x{} ({} bytes) != {}x{} ({} bytes)",
            left.width,
            left.height,
            left.data.len(),
            right.width,
            right.height,
            right.data.len()
        );
    }

    Ok(())
}
