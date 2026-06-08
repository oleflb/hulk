mod feature_extractor;
mod odometry;
mod triangulator;

use std::{
    boxed::Box,
    future::Future,
    path::Path,
    pin::Pin,
    sync::Arc,
    time::{Duration, Instant},
};

use color_eyre::Result;
use nalgebra as na;

use ros_z::prelude::*;
use ros_z::qos::QosDurability;
use types::{
    parameters::StereoVisualOdometryParameters, stereo_camera_info::StereoCameraInfo,
    stereo_image_pair::StereoImagePair, time_wrapper::TimeWrapper,
};

use crate::{
    feature_extractor::{FeatureExtractor, KEYPOINTS, PreviousFeatureState},
    odometry::{OdometryScratch, PreviousFrame, estimate_previous_to_current},
    triangulator::StereoTriangulator,
};

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx))
}

async fn run(ctx: Arc<Context>) -> Result<()> {
    let node = ctx.create_node("stereo_visual_odometry").build().await?;
    let node_parameters =
        node.bind_parameter_as::<StereoVisualOdometryParameters>("stereo_visual_odometry")?;
    let mut parameters_receiver = node_parameters.subscribe();

    let stereo_camera_info_cache = node
        .create_cache::<StereoCameraInfo>("inputs/stereo_camera_info", 1)?
        .with_qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        })
        .build()
        .await?;

    let stereo_image_pair_sub = node
        .subscriber::<TimeWrapper<StereoImagePair>>("inputs/stereo_image_pair")?
        .build()
        .await?;

    let feature_duration_pub = node
        .publisher::<Duration>("visual_odometry/feature_extraction_duration")?
        .build()
        .await?;
    let odometry_pub = node
        .publisher::<na::Isometry3<f32>>(
            "visual_odometry/previous_left_camera_to_current_left_camera",
        )?
        .build()
        .await?;

    let mut pipeline = None;

    loop {
        while !parameters_receiver.borrow().typed().enable {
            parameters_receiver.changed().await?;
        }

        let stereo_image_pair = stereo_image_pair_sub.recv().await?.inner;
        let stereo_camera_info = stereo_camera_info_cache.get_latest();
        let parameters_snapshot = node_parameters.snapshot();
        let parameters = parameters_snapshot.typed();
        if !parameters.enable {
            continue;
        }

        let active_pipeline = match pipeline.take() {
            Some(pipeline) => pipeline,
            None => VisualOdometryPipeline::new(
                parameters
                    .neural_networks_folder
                    .join(&parameters.model_name),
            )?,
        };

        let (next_pipeline, odometry, duration) = tokio::task::spawn_blocking(move || {
            let start_time = Instant::now();
            let mut pipeline = active_pipeline;
            let odometry = pipeline.process(stereo_image_pair, stereo_camera_info.as_deref())?;
            Result::<_>::Ok((pipeline, odometry, start_time.elapsed()))
        })
        .await??;
        pipeline = Some(next_pipeline);
        if !node_parameters.snapshot().typed().enable {
            continue;
        }

        if let Some(odometry) = odometry {
            odometry_pub.publish(&odometry).await?;
        }
        feature_duration_pub.publish(&duration).await?;
    }
}

struct VisualOdometryPipeline {
    feature_extractor: FeatureExtractor,
    triangulator: Option<StereoTriangulator>,
    previous_features: PreviousFeatureState,
    previous_frame: Option<PreviousFrame>,
    current_points: Vec<crate::triangulator::StereoPoint>,
    odometry_scratch: OdometryScratch,
}

impl VisualOdometryPipeline {
    fn new(model_path: impl AsRef<Path>) -> Result<Self> {
        Ok(Self {
            feature_extractor: FeatureExtractor::new(model_path)?,
            triangulator: None,
            previous_features: PreviousFeatureState::new(),
            previous_frame: None,
            current_points: Vec::with_capacity(KEYPOINTS),
            odometry_scratch: OdometryScratch::new(),
        })
    }

    fn process(
        &mut self,
        stereo_image_pair: StereoImagePair,
        stereo_camera_info: Option<&StereoCameraInfo>,
    ) -> Result<Option<na::Isometry3<f32>>> {
        if self.triangulator.is_none() {
            let Some(stereo_camera_info) = stereo_camera_info else {
                return Ok(None);
            };
            self.triangulator = Some(StereoTriangulator::new(
                &stereo_camera_info.left,
                &stereo_camera_info.right,
            )?);
        }
        let Some(triangulator) = self.triangulator.as_ref() else {
            return Ok(None);
        };

        let odometry = {
            let features = self
                .feature_extractor
                .extract(&stereo_image_pair, &self.previous_features)?;
            let current_left = features.current_left()?;
            let current_right = features.current_right()?;
            let stereo_matches = features.stereo_matches()?;

            triangulator.triangulate_into(
                &current_left,
                &current_right,
                &stereo_matches,
                &mut self.current_points,
            );

            if let Some(previous_frame) = self.previous_frame.as_ref() {
                let temporal_matches = features.temporal_matches()?;
                let odometry = estimate_previous_to_current(
                    previous_frame,
                    &current_left,
                    &temporal_matches,
                    triangulator,
                    &mut self.odometry_scratch,
                );
                features.copy_current_left_to(&mut self.previous_features)?;
                odometry
            } else {
                features.copy_current_left_to(&mut self.previous_features)?;
                None
            }
        };

        if let Some(previous_frame) = self.previous_frame.as_mut() {
            previous_frame.replace_stereo_points(&self.current_points);
        } else {
            self.previous_frame = Some(PreviousFrame::from_stereo_points(&self.current_points));
        }

        Ok(odometry)
    }
}
