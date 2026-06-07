mod feature_extractor;
mod odometry;
mod triangulator;

use std::{
    boxed::Box,
    future::Future,
    pin::Pin,
    sync::Arc,
    time::{Duration, Instant},
};

use color_eyre::Result;
use nalgebra as na;

use ros_z::prelude::*;
use ros_z::qos::QosDurability;
use types::{
    stereo_camera_info::StereoCameraInfo, stereo_image_pair::StereoImagePair,
    time_wrapper::TimeWrapper,
};

use crate::{
    feature_extractor::{FeatureExtractor, KEYPOINTS},
    odometry::{OdometryScratch, PreviousFrame, estimate_previous_to_current},
    triangulator::StereoTriangulator,
};

const FEATURE_MODEL_PATH: &str = "etc/neural_networks/xfeat-lighterglue.onnx";

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx))
}

async fn run(ctx: Arc<Context>) -> Result<()> {
    let node = ctx.create_node("stereo_visual_odometry").build().await?;

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
        .publisher::<na::Isometry3<f32>>("visual_odometry/previous_head_to_current_head")?
        .build()
        .await?;

    let mut feature_extractor = FeatureExtractor::new(FEATURE_MODEL_PATH)?;
    let mut triangulator = None;
    let mut previous_stereo = None;
    let mut previous_frame = None;
    let mut current_points = Vec::with_capacity(KEYPOINTS);
    let mut odometry_scratch = OdometryScratch::new();

    loop {
        let stereo_image_pair = stereo_image_pair_sub.recv().await?.inner;
        if triangulator.is_none() {
            let Some(stereo_camera_info) = stereo_camera_info_cache.get_latest() else {
                continue;
            };
            triangulator = Some(StereoTriangulator::new(
                &stereo_camera_info.left,
                &stereo_camera_info.right,
            )?);
        }
        let Some(triangulator) = triangulator.as_ref() else {
            continue;
        };

        let start_time = Instant::now();
        let odometry = {
            let reference_stereo = previous_stereo.as_ref().unwrap_or(&stereo_image_pair);
            let features = feature_extractor.extract(reference_stereo, &stereo_image_pair)?;
            let current_left = features.current_left()?;
            let current_right = features.current_right()?;
            let stereo_matches = features.stereo_matches()?;

            triangulator.triangulate_into(
                &current_left,
                &current_right,
                &stereo_matches,
                &mut current_points,
            );

            if let Some(previous_frame) = previous_frame.as_ref() {
                let temporal_matches = features.temporal_matches()?;
                estimate_previous_to_current(
                    previous_frame,
                    &current_left,
                    &temporal_matches,
                    triangulator,
                    &mut odometry_scratch,
                )
            } else {
                None
            }
        };
        let duration = start_time.elapsed();

        if let Some(previous_frame) = previous_frame.as_mut() {
            previous_frame.replace_stereo_points(&current_points);
        } else {
            previous_frame = Some(PreviousFrame::from_stereo_points(&current_points));
        }
        previous_stereo = Some(stereo_image_pair);

        if let Some(odometry) = odometry {
            odometry_pub.publish(&odometry).await?;
        }
        feature_duration_pub.publish(&duration).await?;
    }
}
