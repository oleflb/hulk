use std::{marker::PhantomData, path::Path, time::Duration};

use color_eyre::{
    Result,
    eyre::{ContextCompat, bail, ensure},
};

use ort::{
    execution_providers::{CUDAExecutionProvider, TensorRTExecutionProvider},
    inputs,
    session::{
        HasSelectedOutputs, RunOptions, Session, SessionOutputs, builder::GraphOptimizationLevel,
        run_options::OutputSelector,
    },
    tensor::PrimitiveTensorElementType,
    value::TensorRef,
};
use ros2::sensor_msgs::image::Image;
use types::stereo_image_pair::StereoImagePair;

use crate::parameters::StereoVisualOdometryPoseEstimationParameters;

pub const NUM_KEYPOINTS: usize = 192;
const DESCRIPTOR_DIMENSION: usize = 64;

pub struct FeatureExtractor {
    session: Session,
    run_options: RunOptions<HasSelectedOutputs>,
    model: FeatureModel,
    batched_images: Vec<u8>,
}

pub struct FeatureOutput<'a> {
    outputs: SessionOutputs<'a>,
    model: FeatureModel,
    timings: FeatureExtractionTimings,
    stereo_matches: Vec<i32>,
    stereo_scores: Vec<f32>,
    temporal_matches: Vec<i32>,
    temporal_scores: Vec<f32>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct FeatureExtractionTimings {
    pub inference: Duration,
    pub matching: Duration,
    pub total: Duration,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FeatureModel {
    FusedLighterGlue,
    StereoLighterGlue,
    XFeatOnly,
}

pub struct PreviousFeatureState {
    keypoints: Vec<f32>,
    descriptors: Vec<f32>,
    valid: Vec<bool>,
}

#[derive(Clone, Copy, Debug)]
pub struct PreviousLeft;

#[derive(Clone, Copy, Debug)]
pub struct CurrentLeft;

#[derive(Clone, Copy, Debug)]
pub struct CurrentRight;

#[derive(Clone, Copy, Debug)]
pub struct FrameFeatures<'a, Frame> {
    keypoints: &'a [f32],
    valid: &'a [bool],
    _frame: PhantomData<Frame>,
}

#[derive(Clone, Copy, Debug)]
pub struct FrameKeypoints<'a, Frame> {
    keypoints: &'a [f32],
    _frame: PhantomData<Frame>,
}

#[derive(Clone, Copy, Debug)]
pub struct Matches<'a, From, To> {
    matches: &'a [i32],
    scores: &'a [f32],
    _frames: PhantomData<(From, To)>,
}

#[derive(Clone, Copy)]
struct FeatureSlices<'a> {
    keypoints: &'a [f32],
    descriptors: &'a [f32],
    valid: &'a [bool],
}

#[derive(Clone, Copy)]
struct BestDescriptorMatch {
    index: usize,
    score: f32,
    second_score: f32,
}

#[derive(Clone, Copy)]
enum DescriptorMatchGeometry {
    Stereo {
        max_vertical_px: f32,
        max_disparity_px: f32,
    },
    Temporal {
        max_distance_px: f32,
    },
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
        let cuda = CUDAExecutionProvider::default().build();
        let session = Session::builder()?
            .with_execution_providers([tensorrt, cuda])?
            .with_optimization_level(GraphOptimizationLevel::Level3)?
            .with_intra_threads(2)?
            .commit_from_file(path)?;
        let model = FeatureModel::from_inputs(&session)?;

        let run_options = RunOptions::new()?.with_outputs(model.output_selector());

        Ok(Self {
            session,
            run_options,
            model,
            batched_images: Vec::new(),
        })
    }

    pub fn extract<'a>(
        &'a mut self,
        current: &StereoImagePair,
        previous: &PreviousFeatureState,
        parameters: &StereoVisualOdometryPoseEstimationParameters,
    ) -> Result<FeatureOutput<'a>> {
        check_stereo_pair_support(current)?;

        let inference_start = std::time::Instant::now();
        let outputs = match self.model {
            FeatureModel::FusedLighterGlue => {
                let current_left = image_tensor(&current.left)?;
                let current_right = image_tensor(&current.right)?;
                let previous_left_keypoints = previous.keypoints_tensor()?;
                let previous_left_descriptors = previous.descriptors_tensor()?;
                let previous_left_valid = previous.valid_tensor()?;

                self.session.run_with_options(
                    inputs![
                        "current_left" => current_left,
                        "current_right" => current_right,
                        "previous_left_keypoints" => previous_left_keypoints,
                        "previous_left_descriptors" => previous_left_descriptors,
                        "previous_left_valid" => previous_left_valid,
                    ],
                    &self.run_options,
                )?
            }
            FeatureModel::StereoLighterGlue => {
                let current_left = image_tensor(&current.left)?;
                let current_right = image_tensor(&current.right)?;

                self.session.run_with_options(
                    inputs![
                        "current_left" => current_left,
                        "current_right" => current_right,
                    ],
                    &self.run_options,
                )?
            }
            FeatureModel::XFeatOnly => {
                self.batched_images.clear();
                self.batched_images
                    .extend_from_slice(current.left.data.as_ref());
                self.batched_images
                    .extend_from_slice(current.right.data.as_ref());
                let image_shape = [
                    2,
                    current.left.height as usize / 2,
                    current.left.width as usize / 2,
                    6,
                ];
                let images =
                    TensorRef::from_array_view((image_shape, self.batched_images.as_slice()))?;
                self.session
                    .run_with_options(inputs!["raw_bytes_input" => images], &self.run_options)?
            }
        };
        let inference = inference_start.elapsed();

        let matching_start = std::time::Instant::now();
        let (stereo_matches, stereo_scores, temporal_matches, temporal_scores) = match self.model {
            FeatureModel::FusedLighterGlue => (Vec::new(), Vec::new(), Vec::new(), Vec::new()),
            FeatureModel::StereoLighterGlue => {
                stereo_lighterglue_temporal_matches(&outputs, previous, current, parameters)?
            }
            FeatureModel::XFeatOnly => xfeat_only_matches(&outputs, previous, current, parameters)?,
        };
        let matching = matching_start.elapsed();

        Ok(FeatureOutput {
            outputs,
            model: self.model,
            timings: FeatureExtractionTimings {
                inference,
                matching,
                total: inference + matching,
            },
            stereo_matches,
            stereo_scores,
            temporal_matches,
            temporal_scores,
        })
    }
}

impl FeatureModel {
    fn from_inputs(session: &Session) -> Result<Self> {
        let has_input = |name: &str| session.inputs.iter().any(|input| input.name == name);

        if has_input("current_left") && has_input("current_right") {
            if has_input("previous_left_keypoints") {
                return Ok(Self::FusedLighterGlue);
            }
            return Ok(Self::StereoLighterGlue);
        }

        if has_input("raw_bytes_input") {
            return Ok(Self::XFeatOnly);
        }

        bail!(
            "unsupported visual odometry model inputs: {:?}",
            session
                .inputs
                .iter()
                .map(|input| input.name.as_str())
                .collect::<Vec<_>>()
        )
    }

    fn output_selector(self) -> OutputSelector {
        match self {
            Self::FusedLighterGlue => OutputSelector::no_default()
                .with("current_left_keypoints")
                .with("current_left_descriptors")
                .with("current_left_valid")
                .with("current_right_keypoints")
                .with("stereo_matches")
                .with("stereo_scores")
                .with("temporal_matches")
                .with("temporal_scores"),
            Self::StereoLighterGlue => OutputSelector::no_default()
                .with("current_left_keypoints")
                .with("current_left_descriptors")
                .with("current_left_valid")
                .with("current_right_keypoints")
                .with("stereo_matches")
                .with("stereo_scores"),
            Self::XFeatOnly => OutputSelector::no_default()
                .with("keypoints")
                .with("descriptors")
                .with("valid"),
        }
    }
}

impl PreviousFeatureState {
    pub fn new() -> Self {
        Self {
            keypoints: vec![0.0; NUM_KEYPOINTS * 2],
            descriptors: vec![0.0; NUM_KEYPOINTS * DESCRIPTOR_DIMENSION],
            valid: vec![false; NUM_KEYPOINTS],
        }
    }

    fn replace(&mut self, keypoints: &[f32], descriptors: &[f32], valid: &[bool]) {
        self.keypoints.copy_from_slice(keypoints);
        self.descriptors.copy_from_slice(descriptors);
        self.valid.copy_from_slice(valid);
    }

    fn keypoints_tensor(&self) -> Result<TensorRef<'_, f32>> {
        TensorRef::from_array_view(([NUM_KEYPOINTS, 2], self.keypoints.as_slice()))
            .map_err(Into::into)
    }

    fn descriptors_tensor(&self) -> Result<TensorRef<'_, f32>> {
        TensorRef::from_array_view((
            [NUM_KEYPOINTS, DESCRIPTOR_DIMENSION],
            self.descriptors.as_slice(),
        ))
        .map_err(Into::into)
    }

    fn valid_tensor(&self) -> Result<TensorRef<'_, bool>> {
        TensorRef::from_array_view(([NUM_KEYPOINTS], self.valid.as_slice())).map_err(Into::into)
    }
}

impl<'a> FeatureOutput<'a> {
    pub fn timings(&self) -> FeatureExtractionTimings {
        self.timings
    }

    pub fn current_left(&self) -> Result<FrameFeatures<'_, CurrentLeft>> {
        match self.model {
            FeatureModel::FusedLighterGlue | FeatureModel::StereoLighterGlue => {
                self.frame("current_left_keypoints", "current_left_valid")
            }
            FeatureModel::XFeatOnly => self.batched_frame(0),
        }
    }

    pub fn current_right(&self) -> Result<FrameKeypoints<'_, CurrentRight>> {
        match self.model {
            FeatureModel::FusedLighterGlue | FeatureModel::StereoLighterGlue => {
                self.keypoints("current_right_keypoints")
            }
            FeatureModel::XFeatOnly => self.batched_keypoints(1),
        }
    }

    pub fn stereo_matches(&self) -> Result<Matches<'_, CurrentLeft, CurrentRight>> {
        match self.model {
            FeatureModel::FusedLighterGlue | FeatureModel::StereoLighterGlue => {
                self.matches("stereo_matches", "stereo_scores")
            }
            FeatureModel::XFeatOnly => Ok(Matches {
                matches: &self.stereo_matches,
                scores: &self.stereo_scores,
                _frames: PhantomData,
            }),
        }
    }

    pub fn temporal_matches(&self) -> Result<Matches<'_, PreviousLeft, CurrentLeft>> {
        match self.model {
            FeatureModel::FusedLighterGlue => self.matches("temporal_matches", "temporal_scores"),
            FeatureModel::StereoLighterGlue => Ok(Matches {
                matches: &self.temporal_matches,
                scores: &self.temporal_scores,
                _frames: PhantomData,
            }),
            FeatureModel::XFeatOnly => Ok(Matches {
                matches: &self.temporal_matches,
                scores: &self.temporal_scores,
                _frames: PhantomData,
            }),
        }
    }

    pub fn copy_current_left_to(&self, state: &mut PreviousFeatureState) -> Result<()> {
        let (keypoints, descriptors, valid) = match self.model {
            FeatureModel::FusedLighterGlue | FeatureModel::StereoLighterGlue => (
                self.tensor::<f32>("current_left_keypoints")?,
                self.tensor::<f32>("current_left_descriptors")?,
                self.tensor::<bool>("current_left_valid")?,
            ),
            FeatureModel::XFeatOnly => (
                self.batched_tensor::<f32>("keypoints", 0, 2)?,
                self.batched_tensor::<f32>("descriptors", 0, DESCRIPTOR_DIMENSION)?,
                self.batched_tensor::<bool>("valid", 0, 1)?,
            ),
        };

        ensure!(
            keypoints.len() == NUM_KEYPOINTS * 2,
            "unexpected current_left_keypoints length: {}",
            keypoints.len()
        );
        ensure!(
            descriptors.len() == NUM_KEYPOINTS * DESCRIPTOR_DIMENSION,
            "unexpected current_left_descriptors length: {}",
            descriptors.len()
        );
        ensure!(
            valid.len() == NUM_KEYPOINTS,
            "unexpected current_left_valid length: {}",
            valid.len()
        );

        state.replace(keypoints, descriptors, valid);
        Ok(())
    }

    fn keypoints<Frame>(&self, keypoints_name: &str) -> Result<FrameKeypoints<'_, Frame>> {
        let keypoints = self.tensor::<f32>(keypoints_name)?;

        ensure!(
            keypoints.len() == NUM_KEYPOINTS * 2,
            "unexpected {keypoints_name} length: {}",
            keypoints.len()
        );

        Ok(FrameKeypoints {
            keypoints,
            _frame: PhantomData,
        })
    }

    fn batched_keypoints<Frame>(&self, batch_index: usize) -> Result<FrameKeypoints<'_, Frame>> {
        Ok(FrameKeypoints {
            keypoints: self.batched_tensor::<f32>("keypoints", batch_index, 2)?,
            _frame: PhantomData,
        })
    }

    fn frame<Frame>(
        &self,
        keypoints_name: &str,
        valid_name: &str,
    ) -> Result<FrameFeatures<'_, Frame>> {
        let keypoints = self.tensor::<f32>(keypoints_name)?;
        let valid = self.tensor::<bool>(valid_name)?;

        ensure!(
            keypoints.len() == NUM_KEYPOINTS * 2,
            "unexpected {keypoints_name} length: {}",
            keypoints.len()
        );
        ensure!(
            valid.len() == NUM_KEYPOINTS,
            "unexpected {valid_name} length: {}",
            valid.len()
        );

        Ok(FrameFeatures {
            keypoints,
            valid,
            _frame: PhantomData,
        })
    }

    fn batched_frame<Frame>(&self, batch_index: usize) -> Result<FrameFeatures<'_, Frame>> {
        Ok(FrameFeatures {
            keypoints: self.batched_tensor::<f32>("keypoints", batch_index, 2)?,
            valid: self.batched_tensor::<bool>("valid", batch_index, 1)?,
            _frame: PhantomData,
        })
    }

    fn matches<From, To>(
        &self,
        matches_name: &str,
        scores_name: &str,
    ) -> Result<Matches<'_, From, To>> {
        let matches = self.tensor::<i32>(matches_name)?;
        let scores = self.tensor::<f32>(scores_name)?;

        ensure!(
            matches.len() == NUM_KEYPOINTS,
            "unexpected {matches_name} length: {}",
            matches.len()
        );
        ensure!(
            scores.len() == NUM_KEYPOINTS,
            "unexpected {scores_name} length: {}",
            scores.len()
        );

        Ok(Matches {
            matches,
            scores,
            _frames: PhantomData,
        })
    }

    fn tensor<T: PrimitiveTensorElementType>(&self, name: &str) -> Result<&[T]> {
        output_tensor(&self.outputs, name)
    }

    fn batched_tensor<T: PrimitiveTensorElementType>(
        &self,
        name: &str,
        batch_index: usize,
        values_per_keypoint: usize,
    ) -> Result<&[T]> {
        batched_tensor(&self.outputs, name, batch_index, values_per_keypoint)
    }
}

impl<Frame> FrameFeatures<'_, Frame> {
    pub fn keypoint(&self, index: usize) -> Option<[f32; 2]> {
        keypoint(self.keypoints, index)
    }

    pub fn is_valid(&self, index: usize) -> bool {
        self.valid.get(index).copied().unwrap_or(false)
    }
}

impl<Frame> FrameKeypoints<'_, Frame> {
    pub fn keypoint(&self, index: usize) -> Option<[f32; 2]> {
        keypoint(self.keypoints, index)
    }
}

fn keypoint(keypoints: &[f32], index: usize) -> Option<[f32; 2]> {
    let offset = index.checked_mul(2)?;
    Some([*keypoints.get(offset)?, *keypoints.get(offset + 1)?])
}

impl<From, To> Matches<'_, From, To> {
    pub fn left_to_right(&self) -> impl Iterator<Item = (usize, usize, f32)> + '_ {
        self.matches
            .iter()
            .zip(self.scores.iter())
            .enumerate()
            .filter_map(|(left_index, (&right_index, &score))| {
                let right_index = usize::try_from(right_index).ok()?;
                (score > 0.0 && right_index < NUM_KEYPOINTS).then_some((
                    left_index,
                    right_index,
                    score,
                ))
            })
    }
}

fn stereo_lighterglue_temporal_matches(
    outputs: &SessionOutputs<'_>,
    previous: &PreviousFeatureState,
    current: &StereoImagePair,
    parameters: &StereoVisualOdometryPoseEstimationParameters,
) -> Result<(Vec<i32>, Vec<f32>, Vec<i32>, Vec<f32>)> {
    let keypoints = output_tensor::<f32>(outputs, "current_left_keypoints")?;
    let descriptors = output_tensor::<f32>(outputs, "current_left_descriptors")?;
    let valid = output_tensor::<bool>(outputs, "current_left_valid")?;
    ensure!(
        keypoints.len() == NUM_KEYPOINTS * 2,
        "unexpected current_left_keypoints length: {}",
        keypoints.len()
    );
    ensure!(
        descriptors.len() == NUM_KEYPOINTS * DESCRIPTOR_DIMENSION,
        "unexpected current_left_descriptors length: {}",
        descriptors.len()
    );
    ensure!(
        valid.len() == NUM_KEYPOINTS,
        "unexpected current_left_valid length: {}",
        valid.len()
    );

    let previous_left = FeatureSlices {
        keypoints: &previous.keypoints,
        descriptors: &previous.descriptors,
        valid: &previous.valid,
    };
    let current_left = FeatureSlices {
        keypoints,
        descriptors,
        valid,
    };
    let normalized_to_pixel_scale = current.left.width.max(current.left.height) as f32 / 2.0;
    let (temporal_matches, temporal_scores) = match_descriptors(
        previous_left,
        current_left,
        normalized_to_pixel_scale,
        DescriptorMatchGeometry::Temporal {
            max_distance_px: parameters.temporal_match_max_distance_px,
        },
        parameters,
    );

    Ok((Vec::new(), Vec::new(), temporal_matches, temporal_scores))
}

fn xfeat_only_matches(
    outputs: &SessionOutputs<'_>,
    previous: &PreviousFeatureState,
    current: &StereoImagePair,
    parameters: &StereoVisualOdometryPoseEstimationParameters,
) -> Result<(Vec<i32>, Vec<f32>, Vec<i32>, Vec<f32>)> {
    let keypoints = output_tensor::<f32>(outputs, "keypoints")?;
    let descriptors = output_tensor::<f32>(outputs, "descriptors")?;
    let valid = output_tensor::<bool>(outputs, "valid")?;
    ensure!(
        keypoints.len() == 2 * NUM_KEYPOINTS * 2,
        "unexpected keypoints length: {}",
        keypoints.len()
    );
    ensure!(
        descriptors.len() == 2 * NUM_KEYPOINTS * DESCRIPTOR_DIMENSION,
        "unexpected descriptors length: {}",
        descriptors.len()
    );
    ensure!(
        valid.len() == 2 * NUM_KEYPOINTS,
        "unexpected valid length: {}",
        valid.len()
    );

    let current_left = FeatureSlices {
        keypoints: &keypoints[..NUM_KEYPOINTS * 2],
        descriptors: &descriptors[..NUM_KEYPOINTS * DESCRIPTOR_DIMENSION],
        valid: &valid[..NUM_KEYPOINTS],
    };
    let current_right = FeatureSlices {
        keypoints: &keypoints[NUM_KEYPOINTS * 2..],
        descriptors: &descriptors[NUM_KEYPOINTS * DESCRIPTOR_DIMENSION..],
        valid: &valid[NUM_KEYPOINTS..],
    };
    let previous_left = FeatureSlices {
        keypoints: &previous.keypoints,
        descriptors: &previous.descriptors,
        valid: &previous.valid,
    };
    let normalized_to_pixel_scale = current.left.width.max(current.left.height) as f32 / 2.0;

    let (stereo_matches, stereo_scores) = match_descriptors(
        current_left,
        current_right,
        normalized_to_pixel_scale,
        DescriptorMatchGeometry::Stereo {
            max_vertical_px: parameters.max_vertical_disparity_px,
            max_disparity_px: parameters.stereo_match_max_disparity_px,
        },
        parameters,
    );
    let (temporal_matches, temporal_scores) = match_descriptors(
        previous_left,
        current_left,
        normalized_to_pixel_scale,
        DescriptorMatchGeometry::Temporal {
            max_distance_px: parameters.temporal_match_max_distance_px,
        },
        parameters,
    );

    Ok((
        stereo_matches,
        stereo_scores,
        temporal_matches,
        temporal_scores,
    ))
}

fn match_descriptors(
    from: FeatureSlices<'_>,
    to: FeatureSlices<'_>,
    normalized_to_pixel_scale: f32,
    geometry: DescriptorMatchGeometry,
    parameters: &StereoVisualOdometryPoseEstimationParameters,
) -> (Vec<i32>, Vec<f32>) {
    let mut best_to_for_from = vec![None; NUM_KEYPOINTS];
    let mut best_from_for_to = vec![None; NUM_KEYPOINTS];

    for from_index in 0..NUM_KEYPOINTS {
        if !from.valid.get(from_index).copied().unwrap_or(false) {
            continue;
        }
        let Some(from_keypoint) = keypoint(from.keypoints, from_index) else {
            continue;
        };

        for to_index in 0..NUM_KEYPOINTS {
            if !to.valid.get(to_index).copied().unwrap_or(false) {
                continue;
            }
            let Some(to_keypoint) = keypoint(to.keypoints, to_index) else {
                continue;
            };
            if !geometry.allows(from_keypoint, to_keypoint, normalized_to_pixel_scale) {
                continue;
            }

            let score = descriptor_dot(from.descriptors, from_index, to.descriptors, to_index);
            if !score.is_finite() {
                continue;
            }

            update_best_descriptor_match(&mut best_to_for_from[from_index], to_index, score);
            update_best_descriptor_match(&mut best_from_for_to[to_index], from_index, score);
        }
    }

    let mut matches = vec![-1; NUM_KEYPOINTS];
    let mut scores = vec![0.0; NUM_KEYPOINTS];
    for from_index in 0..NUM_KEYPOINTS {
        let Some(best_to) = best_to_for_from[from_index] else {
            continue;
        };
        let Some(best_from) = best_from_for_to[best_to.index] else {
            continue;
        };
        if best_from.index != from_index
            || !descriptor_match_passes(best_to, parameters)
            || !descriptor_match_passes(best_from, parameters)
        {
            continue;
        }

        matches[from_index] = best_to.index as i32;
        scores[from_index] = best_to.score.max(0.0);
    }

    (matches, scores)
}

impl DescriptorMatchGeometry {
    fn allows(
        self,
        from_keypoint: [f32; 2],
        to_keypoint: [f32; 2],
        normalized_to_pixel_scale: f32,
    ) -> bool {
        let delta_x = (from_keypoint[0] - to_keypoint[0]) * normalized_to_pixel_scale;
        let delta_y = (from_keypoint[1] - to_keypoint[1]) * normalized_to_pixel_scale;
        match self {
            Self::Stereo {
                max_vertical_px,
                max_disparity_px,
            } => {
                delta_x.is_finite()
                    && delta_y.is_finite()
                    && delta_x > 0.0
                    && delta_x <= max_disparity_px
                    && delta_y.abs() <= max_vertical_px
            }
            Self::Temporal { max_distance_px } => {
                if max_distance_px <= 0.0 {
                    return true;
                }
                delta_x.is_finite()
                    && delta_y.is_finite()
                    && delta_x.mul_add(delta_x, delta_y * delta_y)
                        <= max_distance_px * max_distance_px
            }
        }
    }
}

fn update_best_descriptor_match(slot: &mut Option<BestDescriptorMatch>, index: usize, score: f32) {
    match slot {
        Some(best) if score > best.score => {
            best.second_score = best.score;
            best.score = score;
            best.index = index;
        }
        Some(best) if score > best.second_score => {
            best.second_score = score;
        }
        Some(_) => {}
        None => {
            *slot = Some(BestDescriptorMatch {
                index,
                score,
                second_score: f32::NEG_INFINITY,
            });
        }
    }
}

fn descriptor_match_passes(
    candidate: BestDescriptorMatch,
    parameters: &StereoVisualOdometryPoseEstimationParameters,
) -> bool {
    candidate.score >= parameters.descriptor_match_min_score
        && (candidate.second_score == f32::NEG_INFINITY
            || candidate.score - candidate.second_score >= parameters.descriptor_match_min_margin)
}

fn descriptor_dot(
    from_descriptors: &[f32],
    from_index: usize,
    to_descriptors: &[f32],
    to_index: usize,
) -> f32 {
    let from_start = from_index * DESCRIPTOR_DIMENSION;
    let to_start = to_index * DESCRIPTOR_DIMENSION;
    from_descriptors[from_start..from_start + DESCRIPTOR_DIMENSION]
        .iter()
        .zip(&to_descriptors[to_start..to_start + DESCRIPTOR_DIMENSION])
        .map(|(&from, &to)| from * to)
        .sum()
}

fn output_tensor<'a, T: PrimitiveTensorElementType>(
    outputs: &'a SessionOutputs<'a>,
    name: &str,
) -> Result<&'a [T]> {
    let output = outputs
        .get(name)
        .wrap_err_with(|| format!("missing model output '{name}'"))?;
    let (_, data) = output.try_extract_tensor::<T>()?;
    Ok(data)
}

fn batched_tensor<'a, T: PrimitiveTensorElementType>(
    outputs: &'a SessionOutputs<'a>,
    name: &str,
    batch_index: usize,
    values_per_keypoint: usize,
) -> Result<&'a [T]> {
    ensure!(batch_index < 2, "batch index out of bounds: {batch_index}");
    let values = output_tensor::<T>(outputs, name)?;
    let values_per_batch = NUM_KEYPOINTS * values_per_keypoint;
    ensure!(
        values.len() == 2 * values_per_batch,
        "unexpected {name} length: {}",
        values.len()
    );
    let start = batch_index * values_per_batch;
    Ok(&values[start..start + values_per_batch])
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

    if !(width.is_multiple_of(8) && height.is_multiple_of(8)) {
        bail!(
            "image dimensions must be multiples of 8: {}x{}",
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
