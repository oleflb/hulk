//! Replay current association/localization with recorded solve times and resets.
//! Usage: relocalization_replay recording.mcap [comparison.mcap]
//! `audit_frames_queued` pairs frames with queue admission, not visual commitment.
mod replay;

use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    io::Read,
    path::Path,
    time::{Duration, Instant},
};

use booster::{ImuState, LowState};
use color_eyre::{
    Result,
    eyre::{ensure, eyre},
};
use field_mark_association::{
    AssociationGeometry, AssociationInput, FieldMarkAssociationParameters,
    associate_visual_features, exposure_attitude, find_detected_visual_features,
    geometry_for_image, interpolate_attitude,
};
use kinematics::robot_kinematics::RobotKinematics;
use localization_3d::{Localization, Localization3dParameters, initial_robot_to_local_from_imu};
use mcap::{
    records::Record,
    sans_io::{LinearReadEvent, LinearReader},
};
use ros_z::{
    message::{SerdeCdrCodec, WireDecoder},
    time::Time,
};
use types::{
    camera_geometry::{CameraGeometry, interpolate_camera_geometry},
    fall_detection::FallDetection,
    field_dimensions::FieldDimensions,
    localization::{LocalizationEstimate, LocalizationState, LocalizationStatus},
    object_detection::{Object, RobocupObjectLabel},
    odometry::{KINEMATIC_ODOMETRY_TOPIC, KinematicOdometryDelta},
    time_wrapper::TimeWrapper,
    visual_localization::VisualLocalizationFrame,
    visual_odometry::VisualOdometer,
};

const REQUIRED_TOPICS: &[&str] = &[
    "field_dimensions",
    "inputs/low_state",
    "robot_kinematics",
    "camera_geometry",
    "visual_odometry/current_left_camera_to_visual_odometer",
    "detected_objects",
    "localization/status",
    "debug/solve_diagnostics",
];

fn require_topics(seen: &BTreeSet<String>) -> Result<()> {
    let missing: Vec<_> = REQUIRED_TOPICS
        .iter()
        .filter(|topic| !seen.contains(**topic))
        .collect();
    ensure!(
        missing.is_empty(),
        "missing required topic messages: {missing:?}"
    );
    Ok(())
}

fn decode<T: serde::de::DeserializeOwned>(data: &[u8]) -> Result<T> {
    Ok(SerdeCdrCodec::<T>::deserialize(data)?)
}

enum Event {
    KinematicDelta(KinematicOdometryDelta),
    Fall(FallDetection),
    Imu(Time, ImuState),
    Feet(Box<TimeWrapper<RobotKinematics>>),
    Camera(TimeWrapper<CameraGeometry>),
    Vo(VisualOdometer),
    Objects(TimeWrapper<Vec<Object<RobocupObjectLabel>>>),
    Reset(LocalizationStatus),
    Solve,
}
enum Measurement {
    KinematicDelta(KinematicOdometryDelta),
    Imu(Time, ImuState),
    Feet(Box<TimeWrapper<RobotKinematics>>),
    Vo(VisualOdometer),
    Visual(TimeWrapper<VisualLocalizationFrame>),
}
impl Measurement {
    fn time(&self) -> Time {
        match self {
            Self::KinematicDelta(v) => v.time,
            Self::Imu(t, _) => *t,
            Self::Feet(v) => v.time,
            Self::Vo(v) => v.time,
            Self::Visual(v) => v.time,
        }
    }
}
fn camera_at(
    cache: &BTreeMap<Time, TimeWrapper<CameraGeometry>>,
    t: Time,
    max_gap: Duration,
) -> Option<CameraGeometry> {
    interpolate_camera_geometry(
        cache.range(..=t).next_back()?.1,
        cache.range(t..).next()?.1,
        t,
        max_gap,
    )
}

fn nearest_sample<T>(cache: &BTreeMap<Time, T>, time: Time, max_gap: Duration) -> Option<&T> {
    cache
        .range(..=time)
        .next_back()
        .into_iter()
        .chain(cache.range(time..).next())
        .min_by_key(|(stamp, _)| stamp.abs_diff(time))
        .filter(|(stamp, _)| stamp.abs_diff(time) <= max_gap)
        .map(|(_, sample)| sample)
}

fn initialize_from_imu(
    time: Time,
    imu: &ImuState,
    epoch: u64,
    parameters: &Localization3dParameters,
    field: &FieldDimensions,
    cameras: &BTreeMap<Time, TimeWrapper<CameraGeometry>>,
    feet: &BTreeMap<Time, TimeWrapper<RobotKinematics>>,
) -> Option<Result<Localization>> {
    let camera = nearest_sample(cameras, time, parameters.timing.max_camera_gap)?;
    let feet = nearest_sample(feet, time, parameters.timing.startup_kinematics_freshness)?;
    Some(Localization::new(
        time,
        epoch,
        parameters,
        field,
        &camera.inner,
        initial_robot_to_local_from_imu(imu, &feet.inner),
    ))
}

fn tracking_geometry(
    latest: &LocalizationEstimate,
    history: &BTreeMap<Time, LocalizationEstimate>,
    status: &LocalizationStatus,
    image_time: Time,
    max_age: Duration,
) -> Option<AssociationGeometry> {
    // A newer ineligible output invalidates older hints, as in live frame processing.
    let latest = AssociationGeometry::from_estimate(latest, status)?;
    let snapshots = history
        .range(image_time.saturating_sub(max_age)..=image_time)
        .rev()
        .filter_map(|(_, estimate)| AssociationGeometry::from_estimate(estimate, status));
    geometry_for_image(image_time, snapshots, &latest, max_age)
}

type AuditEstimate = (LocalizationEstimate, LocalizationState);

fn position_step(
    before: &AuditEstimate,
    after: &AuditEstimate,
) -> Option<(f64, serde_json::Value)> {
    if before.0.epoch != after.0.epoch {
        return None;
    }
    let a = before.0.robot_to_field?.pose.inner.translation.vector;
    let b = after.0.robot_to_field?.pose.inner.translation.vector;
    let distance = (b - a).xy().norm();
    let endpoint = |value: &AuditEstimate| {
        serde_json::json!({
            "time": value.0.time, "time_ns": value.0.time.as_nanos(),
            "epoch": value.0.epoch, "generation": value.0.generation, "state": value.1,
        })
    };
    Some((
        distance,
        serde_json::json!({
            "distance_m": distance, "elapsed_s": after.0.time.abs_diff(before.0.time).as_secs_f64(),
            "before": endpoint(before), "after": endpoint(after),
        }),
    ))
}

fn jump_diagnostics(output: &[AuditEstimate]) -> serde_json::Value {
    let fields: Vec<_> = output
        .iter()
        .filter(|(estimate, _)| estimate.robot_to_field.is_some())
        .collect();
    let raw = fields
        .windows(2)
        .filter_map(|pair| position_step(pair[0], pair[1]))
        .max_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, details)| details);
    let continuous = output
        .windows(2)
        .filter(|pair| pair[0].0.generation == pair[1].0.generation)
        .filter_map(|pair| position_step(&pair[0], &pair[1]))
        .max_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, details)| details);
    serde_json::json!({
        "raw_same_epoch_field_steps": raw,
        "same_generation_adjacent_outputs": continuous,
        "interpretation": "Raw steps may cross generation replacements or missing field outputs. Adjacent-output steps require the same epoch/generation and two field poses; inspect elapsed_s for publication gaps.",
    })
}
fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::WARN)
        .with_writer(std::io::stderr)
        .init();
    let mut args = std::env::args_os().skip(1);
    let path = args.next().ok_or_else(|| eyre!("missing recording path"))?;
    if let Some(value) = path.to_str()
        && value.starts_with("--")
    {
        return Err(eyre!("unknown replay option {value}"));
    }
    let output_path = args.next();
    ensure!(
        args.next().is_none(),
        "usage: relocalization_replay recording.mcap [comparison.mcap]"
    );
    if let Some(output) = &output_path {
        ensure!(
            !Path::new(output).try_exists()?,
            "output already exists; choose a new comparison path"
        );
    }
    let mut file = File::open(&path)?;
    let mut reader = LinearReader::new();
    let mut topics = BTreeMap::new();
    let mut events = Vec::new();
    let mut seen = BTreeSet::new();
    let mut field = None;
    while let Some(event) = reader.next_event() {
        match event? {
            LinearReadEvent::ReadRequest(n) => {
                let n = file.read(reader.insert(n))?;
                ensure!(n > 0, "MCAP ended before the reader completed");
                reader.notify_read(n);
            }
            LinearReadEvent::Record { opcode, data } => match mcap::parse_record(opcode, data)? {
                Record::Channel(c) => {
                    ensure!(
                        c.topic != replay::ESTIMATE_TOPIC && c.topic != replay::STATUS_TOPIC,
                        "input already contains comparison channels"
                    );
                    topics.insert(c.id, c.topic);
                }
                Record::Message { header, data } => {
                    let topic = topics
                        .get(&header.channel_id)
                        .ok_or_else(|| eyre!("unknown MCAP channel {}", header.channel_id))?;
                    let event = match topic.as_str() {
                        KINEMATIC_ODOMETRY_TOPIC => Some(Event::KinematicDelta(decode(&data)?)),
                        "fall_detection/status" => Some(Event::Fall(decode(&data)?)),
                        "field_dimensions" => {
                            let dimensions: FieldDimensions = decode(&data)?;
                            ensure!(
                                field
                                    .as_ref()
                                    .is_none_or(|previous| previous == &dimensions),
                                "field dimensions changed during recording"
                            );
                            field = Some(dimensions);
                            None
                        }
                        "inputs/low_state" => Some(Event::Imu(
                            Time::from_nanos(i64::try_from(header.publish_time)?),
                            decode::<LowState>(&data)?.imu_state,
                        )),
                        "robot_kinematics" => Some(Event::Feet(Box::new(decode(&data)?))),
                        "camera_geometry" => Some(Event::Camera(decode(&data)?)),
                        "visual_odometry/current_left_camera_to_visual_odometer" => {
                            Some(Event::Vo(decode(&data)?))
                        }
                        "detected_objects" => Some(Event::Objects(decode(&data)?)),
                        "localization/status" => Some(Event::Reset(decode(&data)?)),
                        "debug/solve_diagnostics" => Some(Event::Solve),
                        _ => continue,
                    };
                    seen.insert(topic.clone());
                    if let Some(event) = event {
                        events.push((header.log_time, event));
                    }
                }
                _ => (),
            },
        }
    }
    require_topics(&seen)?;
    let reconstruct_odometry = !seen.contains(KINEMATIC_ODOMETRY_TOPIC);
    ensure!(
        !reconstruct_odometry || seen.contains("fall_detection/status"),
        "kinematic replay incomplete: no odometry/kinematic_delta and no fall_detection/status for reconstruction"
    );
    let mut odometry = replay::odometry::OdometryReplay::new()?;
    let mut kinematic_deltas = 0;
    let mut admitted_kinematic_deltas = 0;
    let mut comparison = Vec::new();
    let field = field.ok_or_else(|| eyre!("missing field_dimensions"))?;
    events.sort_by_key(|e| e.0);
    let parameters: Localization3dParameters = json5::from_str(include_str!(
        "../../../../etc/parameters/base/localization3d.json5"
    ))?;
    let association_parameters: FieldMarkAssociationParameters = json5::from_str(include_str!(
        "../../../../etc/parameters/base/field_mark_association.json5"
    ))?;
    let mut active: Option<Localization> = None;
    let mut reset: Option<LocalizationStatus> = None;
    let mut cameras = BTreeMap::new();
    let mut attitudes = BTreeMap::new();
    let mut feet = BTreeMap::new();
    let mut initialization_not_ready = 0;
    let mut invalid_initializations = 0;
    let mut detection_camera_unavailable = 0;
    let mut detection_geometry_unavailable = 0;
    let mut vo_camera_unavailable = 0;
    let mut admitted_vo = 0;
    let mut pending = Vec::new();
    let mut estimates: BTreeMap<Time, LocalizationEstimate> = BTreeMap::new();
    let mut output = Vec::new();
    let mut audit_frames_queued = Vec::new();
    let mut audit_solves = Vec::new();
    let mut solve_us = Vec::new();
    let mut association_us = Vec::new();
    let mut previous_status = None;
    let mut lifecycle = Vec::new();
    let started = Instant::now();
    for (delivery, event) in events {
        let now = Time::from_nanos(i64::try_from(delivery)?);
        match event {
            Event::Fall(fall) => odometry.falls.insert(now, fall),
            Event::KinematicDelta(delta) => {
                kinematic_deltas += 1;
                pending.push(Measurement::KinematicDelta(delta));
            }
            Event::Reset(s) => {
                if reset
                    .as_ref()
                    .is_none_or(|previous| previous.epoch != s.epoch)
                {
                    reset = Some(s);
                    active = None;
                    pending.clear();
                    estimates.clear();
                }
            }
            Event::Camera(c) => {
                cameras.insert(c.time, c);
                cameras.retain(|t, _| t.saturating_add(Duration::from_secs(4)) >= now);
            }
            Event::Feet(f) => {
                if reconstruct_odometry {
                    odometry.feet.insert(f.time, (*f).clone());
                }
                feet.insert(f.time, (*f).clone());
                feet.retain(|t, _| t.saturating_add(Duration::from_secs(4)) >= now);
                pending.push(Measurement::Feet(f));
            }
            Event::Imu(t, imu) => {
                if reconstruct_odometry && let Some(delta) = odometry.update(t, now, imu) {
                    kinematic_deltas += 1;
                    pending.push(Measurement::KinematicDelta(delta));
                }
                let rpy = imu.roll_pitch_yaw.inner;
                if let Some(attitude) = exposure_attitude(rpy.x, rpy.y, rpy.z) {
                    attitudes.insert(
                        t,
                        TimeWrapper {
                            time: t,
                            inner: attitude,
                        },
                    );
                }
                attitudes.retain(|t, _| t.saturating_add(Duration::from_secs(4)) >= now);
                pending.push(Measurement::Imu(t, imu));
            }
            Event::Vo(v) => pending.push(Measurement::Vo(v)),
            Event::Objects(objects) => {
                // No field association is possible before initialization.
                let Some(localization) = active.as_mut() else {
                    continue;
                };
                localization.advance_time(now);
                let status = localization.status();
                let epoch = reset
                    .as_ref()
                    .ok_or_else(|| eyre!("detections before initial status"))?;
                let t = objects.time;
                if t < epoch.time || t.saturating_add(Duration::from_millis(250)) < now {
                    continue;
                }
                let Some(camera) = camera_at(&cameras, t, association_parameters.max_camera_gap)
                else {
                    detection_camera_unavailable += 1;
                    continue;
                };
                let attitude = attitudes
                    .range(..=t)
                    .next_back()
                    .zip(attitudes.range(t..).next())
                    .and_then(|((_, before), (_, after))| {
                        interpolate_attitude(before, after, t, association_parameters.max_imu_gap)
                    });
                let geometry = output.last().and_then(|(latest, _)| {
                    tracking_geometry(
                        latest,
                        &estimates,
                        &status,
                        t,
                        association_parameters.max_pose_hint_age,
                    )
                });
                if attitude.is_none() && geometry.is_none() {
                    detection_geometry_unavailable += 1;
                    continue;
                }
                let features = find_detected_visual_features(&objects.inner);
                let timer = Instant::now();
                let result = associate_visual_features(
                    AssociationInput {
                        status: &status,
                        attitude,
                        tracking: geometry.as_ref(),
                        visual_features: &features,
                        robot_to_camera: camera.robot_to_camera,
                        camera_intrinsic: camera.intrinsics,
                        field_dimensions: &field,
                        time: t,
                    },
                    &association_parameters,
                );
                association_us.push(timer.elapsed().as_secs_f64() * 1e6);
                if let Some(result) = result {
                    pending.push(Measurement::Visual(TimeWrapper {
                        time: t,
                        inner: VisualLocalizationFrame {
                            epoch: epoch.epoch,
                            generation: status.generation,
                            source: result.source,
                            robot_to_camera: camera.robot_to_camera,
                            camera_intrinsic: camera.intrinsics,
                            associations: result.associations,
                        },
                    }));
                }
            }
            Event::Solve => {
                let epoch = reset
                    .as_ref()
                    .ok_or_else(|| eyre!("solve before initial localization/status"))?;
                let timer = Instant::now();
                pending.sort_by_key(Measurement::time);
                for m in pending.drain(..) {
                    let t = m.time();
                    if t < epoch.time || t > now {
                        continue;
                    }
                    let l = if let Some(localization) = active.as_mut() {
                        localization
                    } else {
                        let Measurement::Imu(_, ref imu) = m else {
                            continue;
                        };
                        let Some(initialized) = initialize_from_imu(
                            t,
                            imu,
                            epoch.epoch,
                            &parameters,
                            &field,
                            &cameras,
                            &feet,
                        ) else {
                            initialization_not_ready += 1;
                            continue;
                        };
                        match initialized {
                            Ok(localization) => active.insert(localization),
                            Err(error) => {
                                invalid_initializations += 1;
                                tracing::warn!(%error, "discarding invalid initialization sample");
                                continue;
                            }
                        }
                    };
                    match m {
                        Measurement::KinematicDelta(delta) => {
                            admitted_kinematic_deltas +=
                                usize::from(l.ingest_kinematic_odometry(delta)?);
                        }
                        Measurement::Imu(t, imu) => {
                            l.ingest_imu(t, imu)?;
                        }
                        Measurement::Feet(f) => {
                            l.ingest_kinematics(*f)?;
                        }
                        Measurement::Visual(v) => {
                            let audit = v.clone();
                            let queued = l.ingest_visual_localization_frame(v)?;
                            audit_frames_queued.push((audit, queued));
                        }
                        Measurement::Vo(v) => {
                            let previous = v.delta.as_ref().and_then(|d| {
                                camera_at(
                                    &cameras,
                                    d.previous_time,
                                    parameters.timing.max_camera_gap,
                                )
                            });
                            let current =
                                camera_at(&cameras, v.time, parameters.timing.max_camera_gap);
                            if v.delta.is_some() && (previous.is_none() || current.is_none()) {
                                vo_camera_unavailable += 1;
                            }
                            admitted_vo += usize::from(l.ingest_visual_odometry(
                                v,
                                previous.as_ref(),
                                current.as_ref(),
                            )?);
                        }
                    }
                }
                if let Some(l) = active.as_mut() {
                    let ingestion_duration = timer.elapsed();
                    let mut solved = l.solve(now);
                    solved.diagnostics.ingestion_duration = ingestion_duration;
                    let status = l.status();
                    debug_assert!(
                        solved.estimate.is_some() || solved.diagnostics.failure.is_some()
                    );
                    audit_solves.push(serde_json::json!({
                        "time": now, "status": status, "has_estimate": solved.estimate.is_some(),
                        "missing_estimate_reason": solved.diagnostics.failure,
                        "diagnostics": solved.diagnostics,
                    }));
                    comparison.push(replay::Comparison {
                        delivery,
                        status,
                        estimate: solved.estimate,
                    });
                    let key = (status.epoch, status.generation, status.state);
                    if previous_status != Some(key) {
                        lifecycle.push(status);
                        previous_status = Some(key);
                    }
                    if let Some(e) = solved.estimate {
                        output.push((e, status.state));
                        estimates.insert(e.time, e);
                        estimates.retain(|t, _| t.saturating_add(Duration::from_secs(1)) >= now);
                    }
                } else {
                    audit_solves.push(serde_json::json!({
                        "time": now, "status": null, "has_estimate": false,
                        "missing_estimate_reason": "localization_not_initialized", "diagnostics": null,
                    }));
                }
                solve_us.push(timer.elapsed().as_secs_f64() * 1e6);
            }
        }
    }
    debug_assert_eq!(audit_solves.len(), solve_us.len());
    ensure!(!output.is_empty(), "replay produced no estimates");
    ensure!(
        admitted_kinematic_deltas > 0,
        "kinematic replay incomplete: {kinematic_deltas} deltas available, none admitted"
    );
    let kinematic_audit = serde_json::json!({
        "source": if reconstruct_odometry { "reconstructed" } else { "recorded" },
        "available_deltas": kinematic_deltas,
        "admitted_deltas": admitted_kinematic_deltas,
        "missing_aligned_input_updates": odometry.missing_aligned_inputs,
        "incomplete_reconstruction_inputs": reconstruct_odometry && odometry.missing_aligned_inputs > 0,
        "reconstruction_limitation": reconstruct_odometry.then_some(
            "Current base odometry parameters; LowState IMU and recorder delivery order approximate live subscriber scheduling. Missing/stale inputs break delta continuity."
        ),
    });
    if reconstruct_odometry && odometry.missing_aligned_inputs > 0 {
        eprintln!(
            "Kinematic reconstruction has {} missing/stale aligned input updates; those intervals are not fused. See kinematic_odometry in the JSON report and comparison/replay MCAP metadata.",
            odometry.missing_aligned_inputs
        );
    }
    let field_estimates: Vec<_> = output
        .iter()
        .filter_map(|(estimate, _)| estimate.robot_to_field.map(|pose| (estimate, pose)))
        .collect();
    let transitions: Vec<_> = field_estimates
        .windows(2)
        .filter(|pair| pair[0].0.epoch == pair[1].0.epoch)
        .map(|pair| {
            (
                pair[1].0.time.abs_diff(pair[0].0.time).as_secs_f64(),
                (pair[1].1.pose.inner.translation.vector - pair[0].1.pose.inner.translation.vector)
                    .xy()
                    .norm(),
            )
        })
        .collect();
    println!(
        "{}",
        serde_json::to_string_pretty(&serde_json::json!({
            "field_dimensions": field, "wall_s": started.elapsed().as_secs_f64(),
            "kinematic_odometry": kinematic_audit,
            "initialization_not_ready": initialization_not_ready,
            "invalid_initializations": invalid_initializations,
            "detection_camera_unavailable": detection_camera_unavailable,
            "detection_geometry_unavailable": detection_geometry_unavailable,
            "vo_camera_unavailable": vo_camera_unavailable,
            "admitted_vo": admitted_vo,
            "association": timing(&mut association_us), "localization": timing(&mut solve_us),
            "field_estimates": field_estimates.len(),
            "field_height_above_1m": field_estimates.iter().filter(|(_, pose)| pose.pose.inner.translation.z > 1.0).count(),
            "max_field_height_m": field_estimates.iter().map(|(_, pose)| pose.pose.inner.translation.z).reduce(f64::max),
            "longest_field_output_gap_s": transitions.iter().map(|(gap, _)| *gap).reduce(f64::max),
            "largest_position_step_m": transitions.iter().map(|(_, step)| *step).reduce(f64::max),
            "jump_diagnostics": jump_diagnostics(&output),
            "lifecycle": lifecycle, "audit_frames_queued": audit_frames_queued,
            "audit_solves": audit_solves, "audit_estimates": output,
        }))?
    );
    if let Some(output_path) = output_path {
        replay::write_comparison(
            Path::new(&path),
            Path::new(&output_path),
            &comparison,
            &kinematic_audit,
        )?;
    }
    Ok(())
}

fn timing(values: &mut [f64]) -> serde_json::Value {
    values.sort_by(f64::total_cmp);
    if values.is_empty() {
        return serde_json::Value::Null;
    }
    serde_json::json!({"count": values.len(), "total_s": values.iter().sum::<f64>() / 1e6,
        "median_ms": values[values.len() / 2] / 1000.0, "p95_ms": values[values.len() * 95 / 100] / 1000.0})
}

#[cfg(test)]
mod tests {
    use super::*;

    fn field_estimate(nanos: i64, x: f64, generation: u64) -> LocalizationEstimate {
        LocalizationEstimate {
            time: Time::from_nanos(nanos),
            epoch: 0,
            generation,
            robot_to_local: types::localization::PoseEstimate {
                pose: linear_algebra::Isometry3::identity(),
                covariance: nalgebra::SMatrix::identity(),
            },
            robot_to_field: Some(types::localization::PoseEstimate {
                pose: linear_algebra::Isometry3::wrap(nalgebra::Isometry3::translation(
                    x, 0.0, 0.0,
                )),
                covariance: nalgebra::SMatrix::identity(),
            }),
        }
    }

    #[test]
    fn newest_missing_field_pose_invalidates_pre_exposure_tracking_hint() {
        let previous = field_estimate(1_000_000_000, 0.0, 0);
        let mut latest = field_estimate(1_020_000_000, 0.0, 0);
        let image_time = Time::from_nanos(1_010_000_000);
        let status = LocalizationStatus {
            time: previous.time,
            epoch: 0,
            generation: 0,
            state: LocalizationState::Tracking,
            heading: None,
        };
        let history = BTreeMap::from([(previous.time, previous)]);
        let age = Duration::from_millis(250);
        assert!(tracking_geometry(&latest, &history, &status, image_time, age).is_some());
        latest.robot_to_field = None;
        assert!(tracking_geometry(&latest, &history, &status, image_time, age).is_none());
    }

    #[test]
    fn jump_report_separates_generation_replacements_and_missing_outputs() {
        let output = [
            (
                field_estimate(1_000_000_000, 0.0, 0),
                LocalizationState::Tracking,
            ),
            (
                field_estimate(1_010_000_000, 10.0, 1),
                LocalizationState::Tracking,
            ),
            (
                field_estimate(1_020_000_000, 11.0, 1),
                LocalizationState::LostTrack,
            ),
        ];
        let report = jump_diagnostics(&output);
        assert_eq!(report["raw_same_epoch_field_steps"]["distance_m"], 10.0);
        let adjacent = &report["same_generation_adjacent_outputs"];
        assert_eq!(adjacent["distance_m"], 1.0);
        assert_eq!(adjacent["elapsed_s"], 0.01);
        assert_eq!(adjacent["before"]["generation"], 1);
        assert_eq!(adjacent["after"]["state"], "LostTrack");
        assert_eq!(adjacent["after"]["time_ns"], 1_020_000_000_i64);
        let mut missing = output[0];
        missing.0.robot_to_field = None;
        let report = jump_diagnostics(&[
            output[0],
            missing,
            (
                field_estimate(1_020_000_000, 11.0, 0),
                LocalizationState::Tracking,
            ),
        ]);
        assert!(report["same_generation_adjacent_outputs"].is_null());
    }

    #[test]
    fn late_delivered_camera_allows_next_imu_to_initialize_after_unready_solve() -> Result<()> {
        let parameters: Localization3dParameters = json5::from_str(include_str!(
            "../../../../etc/parameters/base/localization3d.json5"
        ))?;
        let first = Time::from_nanos(1_000_000_000);
        let camera_time = first.saturating_add(Duration::from_millis(5));
        let second = first.saturating_add(Duration::from_millis(6));
        let events = [
            Event::Feet(Box::new(TimeWrapper {
                time: first,
                inner: RobotKinematics::default(),
            })),
            Event::Imu(first, ImuState::default()),
            Event::Solve,
            Event::Camera(TimeWrapper {
                time: camera_time,
                inner: CameraGeometry::default(),
            }),
            Event::Imu(second, ImuState::default()),
            Event::Solve,
        ];
        let mut cameras = BTreeMap::new();
        let mut feet = BTreeMap::new();
        let mut pending = None;
        let mut readiness = Vec::new();
        let mut initialized_time = None;
        for event in events {
            match event {
                Event::Feet(sample) => {
                    feet.insert(sample.time, *sample);
                }
                Event::Camera(sample) => {
                    cameras.insert(sample.time, sample);
                }
                Event::Imu(time, imu) => pending = Some((time, imu)),
                Event::Solve => {
                    let (time, imu) = pending.take().ok_or_else(|| eyre!("missing test IMU"))?;
                    let initialized = initialize_from_imu(
                        time,
                        &imu,
                        7,
                        &parameters,
                        &FieldDimensions::SPL_2025,
                        &cameras,
                        &feet,
                    );
                    readiness.push(initialized.is_some());
                    if let Some(localization) = initialized {
                        initialized_time = Some(localization?.status().time);
                    }
                }
                _ => return Err(eyre!("unexpected test event")),
            }
        }
        assert_eq!(readiness, [false, true]);
        assert_eq!(initialized_time, Some(second));
        Ok(())
    }

    #[test]
    fn initialization_uses_nearest_delivered_sample_with_freshness_limit() {
        let time = Time::from_nanos(1_000_000_000);
        let future = time.saturating_add(Duration::from_millis(5));
        let past = time.saturating_sub(Duration::from_millis(30));
        let cache = BTreeMap::from([(past, "stale"), (future, "ready")]);
        assert_eq!(
            nearest_sample(&cache, time, Duration::from_millis(20)),
            Some(&"ready")
        );
        assert!(nearest_sample(&cache, time, Duration::from_millis(4)).is_none());
        assert!(nearest_sample::<()>(&BTreeMap::new(), time, Duration::from_millis(20)).is_none());
    }

    #[test]
    fn every_required_topic_needs_messages() {
        let mut seen: BTreeSet<_> = REQUIRED_TOPICS.iter().map(|s| s.to_string()).collect();
        assert!(require_topics(&seen).is_ok());
        for topic in REQUIRED_TOPICS {
            seen.remove(*topic);
            assert!(require_topics(&seen).is_err_and(|error| error.to_string().contains(topic)));
            seen.insert(topic.to_string());
        }
    }

    #[test]
    fn camera_geometry_has_no_missing_data_fallback() {
        assert!(
            camera_at(
                &BTreeMap::new(),
                Time::from_nanos(1),
                Duration::from_millis(20)
            )
            .is_none()
        );
        let time = Time::from_nanos(1_000_000_000);
        let after = time.saturating_add(Duration::from_millis(30));
        let cache = BTreeMap::from([time, after].map(|time| {
            (
                time,
                TimeWrapper {
                    time,
                    inner: CameraGeometry::default(),
                },
            )
        }));
        assert!(
            camera_at(
                &cache,
                time.saturating_add(Duration::from_millis(5)),
                Duration::from_millis(20)
            )
            .is_none()
        );
        assert!(
            camera_at(
                &cache,
                time.saturating_sub(Duration::from_millis(1)),
                Duration::from_millis(20)
            )
            .is_none()
        );
        assert!(camera_at(&cache, time, Duration::from_millis(20)).is_some());
    }
}
