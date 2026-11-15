use crate::{
    Localization, Localization3dParameters, SolveDiagnostics,
    inputs::{Inputs, Measurement},
    pose::initial_robot_to_local_from_imu,
};
use color_eyre::{Result, eyre::Context as _};
use ros_z::{
    context::Context,
    parameter::NodeParametersExt,
    qos::{QosDurability, QosHistory, QosProfile},
};
use std::{
    future::{Future, pending},
    num::NonZeroUsize,
    pin::Pin,
    sync::Arc,
    time::Duration,
};
use types::{
    field_dimensions::FieldDimensions,
    localization::{LocalizationEstimate, LocalizationState, LocalizationStatus},
    primary_state::PrimaryState,
};

fn solve_timer(period: Duration) -> tokio::time::Interval {
    let mut timer = tokio::time::interval_at(tokio::time::Instant::now() + period, period);
    timer.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    timer
}

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx))
}

pub async fn run(ctx: Arc<Context>) -> Result<()> {
    let node = ctx.create_node("localization3d").build().await?;
    let parameters = node.bind_parameter_as::<Localization3dParameters>("localization3d")?;
    let startup = parameters.snapshot().typed().clone();
    startup
        .validate()
        .map_err(|message| color_eyre::eyre::eyre!(message))?;
    let baseline = startup.clone();
    // Hooks run under the ROSZ commit lock before storage/publication. Rejection
    // leaves both the node snapshot and its subscribers at the previous revision.
    parameters.add_validation_hook(move |candidate| baseline.validate_update(candidate))?;
    let mut updates = parameters.subscribe();
    let inputs = Inputs::new(&node, &startup.inputs).await?;
    let latched = QosProfile {
        durability: QosDurability::TransientLocal,
        history: QosHistory::KeepLast(NonZeroUsize::MIN),
        ..Default::default()
    };
    let primary = node
        .subscriber::<PrimaryState>("primary_state")
        .qos(latched)
        .build()
        .await?;
    let field = node
        .subscriber::<FieldDimensions>("field_dimensions")
        .qos(latched)
        .cache(startup.inputs.field_cache)
        .build()
        .await?;
    let estimates = node
        .publisher::<LocalizationEstimate>("localization/estimate")
        .build()
        .await?;
    let statuses = node
        .publisher::<LocalizationStatus>("localization/status")
        .qos(latched)
        .build()
        .await?;
    let diagnostics = node
        .publisher::<SolveDiagnostics>("debug/solve_diagnostics")
        .build()
        .await?;
    let mut status = LocalizationStatus {
        time: node.clock().now(),
        epoch: 0,
        generation: 0,
        state: LocalizationState::Startup,
        heading: None,
    };
    statuses.publish(&status).await?;
    let mut active: Option<Localization> = None;
    let mut damping = true;
    let mut epoch_start = status.time;
    let mut timer = solve_timer(startup.solver.solve_interval);
    let mut needs_solve = false;
    let mut ingestion_duration = Duration::ZERO;
    loop {
        let deadline = active.as_ref().and_then(Localization::deadline);
        let first = tokio::select! {
            biased;
            v = primary.recv() => {
                let next_damping = v? == PrimaryState::Damping;
                if damping && !next_damping {
                    epoch_start = node.clock().now();
                    active = None;
                    needs_solve = false;
                    ingestion_duration = Duration::ZERO;
                    status = LocalizationStatus { time: epoch_start, epoch: status.epoch.wrapping_add(1), generation: 0, state: LocalizationState::Startup, heading: None };
                    statuses.publish(&status).await?;
                }
                damping = next_damping;
                continue;
            }
            changed = updates.changed() => {
                changed.wrap_err("localization parameters closed")?;
                let snapshot = updates.borrow_and_update().clone();
                if let Some(active) = active.as_mut()
                    && let Err(error) = active.set_parameters(snapshot.typed()) {
                    tracing::error!(%error, "parameter update rejected; retaining active localization");
                    continue;
                }
                if timer.period() != snapshot.typed().solver.solve_interval {
                    timer = solve_timer(snapshot.typed().solver.solve_interval);
                }
                continue;
            }
            _ = async { match deadline { Some(t) => node.clock().sleep_until(t).await, None => pending().await } } => {
                if let Some(active) = active.as_mut() {
                    active.advance_time(node.clock().now());
                    if status != active.status() { status = active.status(); statuses.publish(&status).await?; }
                }
                continue;
            }
            _ = timer.tick() => None,
            sample = inputs.recv() => Some(sample?),
        };
        let Some(first) = first else {
            if !std::mem::take(&mut needs_solve) {
                continue;
            }
            let Some(localization) = active.as_mut() else {
                continue;
            };
            let mut output = tokio::task::block_in_place(|| localization.solve(node.clock().now()));
            output.diagnostics.ingestion_duration = std::mem::take(&mut ingestion_duration);
            // Advance lifecycle after the potentially expensive solve, before publishing.
            localization.advance_time(node.clock().now());
            if status != localization.status() {
                status = localization.status();
                statuses.publish(&status).await?;
            }
            if let Some(estimate) = output.estimate {
                estimates.publish(&estimate).await?;
            }
            if let Some(error) = &output.diagnostics.failure {
                tracing::warn!(%error, "localization solve failed");
            }
            diagnostics.publish(&output.diagnostics).await?;
            continue;
        };
        let samples = inputs.drain(first).await?;
        let now = node.clock().now();
        for sample in samples {
            let time = sample.time();
            if time < epoch_start || time > now {
                tracing::warn!(
                    ?time,
                    "discarding measurement outside current epoch or in the future"
                );
                continue;
            }
            if active
                .as_ref()
                .is_some_and(|localization| localization.has_measurement_gap(time))
            {
                tracing::warn!(
                    ?time,
                    "measurement gap exceeds window; resetting localization epoch"
                );
                active = None;
                // Measurements inserted before the reset belonged to the old estimator.
                needs_solve = false;
                ingestion_duration = Duration::ZERO;
                epoch_start = time;
                status = LocalizationStatus {
                    time,
                    epoch: status.epoch.wrapping_add(1),
                    generation: 0,
                    state: LocalizationState::Startup,
                    heading: None,
                };
                statuses.publish(&status).await?;
            }
            let localization = if let Some(localization) = active.as_mut() {
                localization
            } else {
                let Measurement::Imu(_, imu) = &sample else {
                    continue;
                };
                let Some(camera) = inputs
                    .cameras
                    .get_nearest(time)
                    .filter(|camera| camera.time.abs_diff(time) <= startup.timing.max_camera_gap)
                else {
                    continue;
                };
                let Some(kinematics) = inputs.robot_kinematics.get_nearest(time) else {
                    continue;
                };
                if kinematics.time.abs_diff(time) > startup.timing.startup_kinematics_freshness {
                    continue;
                }
                let Some(field) = field.get_latest() else {
                    continue;
                };
                let initialized = Localization::new(
                    time,
                    status.epoch,
                    updates.borrow().typed(),
                    &field,
                    &camera.inner,
                    initial_robot_to_local_from_imu(imu, &kinematics.inner),
                );
                let localization = match initialized {
                    Ok(localization) => active.insert(localization),
                    Err(error) => {
                        tracing::warn!(%error, "discarding invalid initialization sample");
                        continue;
                    }
                };
                status = localization.status();
                statuses.publish(&status).await?;
                localization
            };
            let started = std::time::Instant::now();
            needs_solve |= inputs.ingest(localization, sample)?;
            ingestion_duration += started.elapsed();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_z::time::Time;

    #[tokio::test(start_paused = true)]
    async fn rejected_update_preserves_running_estimator_and_solve_cadence() -> Result<()> {
        let original: Localization3dParameters = json5::from_str(include_str!(
            "../../../../etc/parameters/base/localization3d.json5"
        ))?;
        let started = tokio::time::Instant::now();
        let mut timer = solve_timer(original.solver.solve_interval);
        assert_eq!(timer.tick().await, started + Duration::from_millis(25));
        tokio::time::advance(Duration::from_millis(100)).await;
        assert_eq!(timer.tick().await, started + Duration::from_millis(50));
        assert_eq!(timer.tick().await, started + Duration::from_millis(150));
        let origin = Time::from_nanos(1_000_000_000);
        let mut active = Localization::new(
            origin,
            7,
            &original,
            &FieldDimensions::SPL_2025,
            &types::camera_geometry::CameraGeometry::default(),
            linear_algebra::Isometry3::identity(),
        )?;
        for millis in (0..=100).step_by(2) {
            active.ingest_imu(
                origin + Duration::from_millis(millis),
                booster::ImuState::default(),
            )?;
        }
        assert!(
            active
                .solve(origin + Duration::from_millis(100))
                .estimate
                .is_some()
        );
        let status = active.status();
        let incompatible_spacing = original.timing.trajectory_spacing * 2;
        for (path, value) in [
            (
                "timing.trajectory_spacing",
                serde_json::to_value(incompatible_spacing)?,
            ),
            ("model.foot_sigma", serde_json::json!(0.02)),
            ("inputs.imu_queue", serde_json::json!(1000)),
            ("inputs.visual_queue", serde_json::json!(120)),
            ("visual.min_associations", serde_json::json!(2)),
        ] {
            let mut candidate = serde_json::to_value(&original)?;
            let (group, key) = path
                .split_once('.')
                .ok_or_else(|| color_eyre::eyre::eyre!("invalid parameter path"))?;
            candidate[group][key] = value;
            let candidate: Localization3dParameters = serde_json::from_value(candidate)?;
            assert!(original.validate_update(&candidate).is_err(), "{path}");
            assert!(active.set_parameters(&candidate).is_err(), "{path}");
            assert_eq!(active.status(), status);
        }
        let mut live = original;
        live.solver.max_iterations = 12;
        live.solver.solve_interval = Duration::from_millis(50);
        active.set_parameters(&live)?;
        timer = solve_timer(live.solver.solve_interval);
        assert_eq!(timer.tick().await, started + Duration::from_millis(200));
        let time = origin + Duration::from_millis(102);
        active.ingest_imu(time, booster::ImuState::default())?;
        let estimate = active
            .solve(time)
            .estimate
            .expect("rejected updates must leave motion running");
        assert_eq!(estimate.time, time);
        assert_eq!(estimate.epoch, 7);
        assert_eq!(estimate.generation, status.generation);
        Ok(())
    }
}
