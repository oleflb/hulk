use std::{collections::BTreeMap, ops::Range};

use color_eyre::{Result, eyre::eyre};
use coordinate_systems::{Field, Local, Robot};
use fagra::{DenseNormalCholesky, LevenbergMarquardt, Problem, StateKey};
use linear_algebra::{Framed, Isometry3, Vector2, Vector3, vector};
use localization_fagra::{
    factors::{
        AdjacentKinematicOdometry, AdjacentVisualOdometry, CameraIntrinsicsPrior, FieldContainment,
        FootGround, FootObservation, FrameReprojections, ImuBiasPrior, ImuBiasWalk, ImuKinematics,
        KinematicOdometry, MotionPrior, PreintegratedImu, ReprojectionObservation, TrajectoryPrior,
        VisualOdometry, VisualOdometryObservation,
    },
    variables::{CameraIntrinsics, FieldAlignment, ImuBias, PoseControl, TrajectoryState},
};
use nalgebra::SMatrix;
use projection::intrinsic::Intrinsic;
use ros_z::time::Time;
use types::{
    field_dimensions::FieldDimensions, time_wrapper::TimeWrapper,
    visual_localization::VisualLocalizationFrame,
};

use crate::{localization::Lifecycle, parameters::Localization3dParameters};
pub(crate) use solve::SolveResult;

mod attitude;
mod bias;
mod covariance;
mod inertial;
mod kinematic_odometry;
mod preintegration;
mod recovery;
mod solve;
mod vision;
mod window;

#[cfg(test)]
mod tests;

fagra::states! {
    States {
        controls: PoseControl,
        alignments: FieldAlignment,
        intrinsics: CameraIntrinsics,
        biases: ImuBias,
    }
}

fagra::factors! {
    Factors {
        trajectory_priors: TrajectoryPrior,
        bias_priors: ImuBiasPrior,
        bias_walks: ImuBiasWalk,
        intrinsics_priors: CameraIntrinsicsPrior,
        motion: MotionPrior,
        yaw: localization_fagra::factors::RelativeYaw,
        containment: FieldContainment,
        adjacent_odometry: AdjacentVisualOdometry,
        kinematic_odometry: KinematicOdometry,
        adjacent_kinematic_odometry: AdjacentKinematicOdometry,
        imu: ImuKinematics,
        preintegrated_imu: PreintegratedImu,
        feet: Batch<FootGround, FootObservation>,
        reprojections: Batch<FrameReprojections, ReprojectionObservation>,
        odometry: Batch<VisualOdometry, VisualOdometryObservation>,
    }
}

type Graph = Problem<States, Factors>;

/// Owns optimization and accepted lifecycle for live and deterministic replay.
pub struct Localization {
    graph: Graph,
    optimizer: LevenbergMarquardt<DenseNormalCholesky>,
    origin: Time,
    pub(super) epoch: u64,
    pub(super) generation: u64,
    controls: BTreeMap<i64, StateKey<PoseControl<f64>>>,
    biases: BTreeMap<i64, StateKey<ImuBias>>,
    preintegration: preintegration::ImuIntervals,
    segment_factors: BTreeMap<i64, window::SegmentFactors>,
    alignment: Option<StateKey<FieldAlignment<f64>>>,
    intrinsics: StateKey<CameraIntrinsics<f64>>,
    pub(super) latest_time: Time,
    last_converged_time: Option<Time>,
    recovery_seed: Option<(Time, nalgebra::Isometry3<f64>)>,
    latest_visual_frame: Option<TimeWrapper<VisualLocalizationFrame>>,
    pending_visuals: Vec<vision::PendingVisual>,
    pub(super) parameters: Localization3dParameters,
    pub(super) lifecycle: Lifecycle,
    field_half_extents: Vector2<Field, f64>,
    history: recovery::MotionHistory,
    kinematic_factors: BTreeMap<Time, (Time, kinematic_odometry::KinematicFactor)>,
}

impl Localization {
    pub(super) fn new_motion(
        origin: Time,
        epoch: u64,
        initial_pose: Isometry3<Robot, Local>,
        intrinsics: &Intrinsic,
        parameters: Localization3dParameters,
        field: &FieldDimensions,
    ) -> Result<Self> {
        let mut estimator = Self::empty(
            origin,
            epoch,
            intrinsics,
            parameters,
            vector![
                field.length as f64 * 0.5 + field.border_strip_width as f64,
                field.width as f64 * 0.5 + field.border_strip_width as f64,
            ],
        )?;
        estimator.initialize_biases(origin, <ImuBias as fagra::Variable>::identity())?;
        let pose = PoseControl {
            pose: Framed::wrap(initial_pose.inner.cast()),
        };
        for index in -1..=2 {
            estimator
                .controls
                .insert(index, estimator.graph.add(pose.clone()));
        }
        estimator.add_anchor(
            origin,
            TrajectoryState {
                pose: pose.pose,
                velocity: Vector3::wrap(nalgebra::Vector3::zeros()),
            },
            Some(estimator.parameters.initial_height_sigma),
        )?;
        estimator.add_motion_prior(0, false)?;
        Ok(estimator)
    }

    fn empty(
        origin: Time,
        epoch: u64,
        camera_intrinsic: &Intrinsic,
        parameters: Localization3dParameters,
        field_half_extents: Vector2<Field, f64>,
    ) -> Result<Self> {
        parameters.validate().map_err(|message| eyre!(message))?;
        let mut graph = Graph::new();
        let intrinsics_value = CameraIntrinsics {
            focal_lengths: Framed::wrap(camera_intrinsic.focals.cast()),
            optical_center: Framed::wrap(camera_intrinsic.optical_center.inner.cast()),
        };
        let intrinsics = graph.add(intrinsics_value.clone());
        graph.add_factor(CameraIntrinsicsPrior {
            intrinsics,
            reference: intrinsics_value,
            information_root: SMatrix::identity() / parameters.model.intrinsic_prior_sigma,
        })?;

        Ok(Self {
            graph,
            optimizer: {
                let mut optimizer = LevenbergMarquardt::new(DenseNormalCholesky::default());
                optimizer.options.max_trials = parameters.solver.max_trials;
                optimizer
            },
            origin,
            epoch,
            generation: 0,
            controls: BTreeMap::new(),
            biases: BTreeMap::new(),
            preintegration: preintegration::ImuIntervals::new(&parameters.timing),
            segment_factors: BTreeMap::new(),
            alignment: None,
            intrinsics,
            latest_time: origin,
            last_converged_time: None,
            recovery_seed: None,
            latest_visual_frame: None,
            pending_visuals: Vec::new(),
            parameters,
            field_half_extents,
            history: recovery::MotionHistory::new(origin),
            kinematic_factors: BTreeMap::new(),
            lifecycle: Lifecycle::new(origin),
        })
    }

    fn add_anchor(
        &mut self,
        time: Time,
        reference: TrajectoryState,
        height_sigma: Option<f64>,
    ) -> Result<()> {
        let (segment, tau) = self.segment_and_tau(time)?;
        let mut root = SMatrix::<f64, 9, 9>::zeros();
        let rotation = reference.pose.inner.rotation.to_rotation_matrix();
        // The residual is in reference body axes. Only Local XY/yaw fix gauge;
        // height and tilt are physical quantities constrained by sensor evidence.
        root.fixed_view_mut::<1, 3>(2, 0)
            .copy_from(&(rotation.matrix().row(2) / self.parameters.model.anchor_yaw_sigma));
        root.fixed_view_mut::<2, 3>(6, 6).copy_from(
            &(rotation.matrix().fixed_rows::<2>(0) / self.parameters.model.anchor_xy_sigma),
        );
        root.fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(rotation.matrix() / self.parameters.initial_velocity_sigma));
        // Broad provisional startup height only; never count a visual seed as
        // another height observation alongside those same reprojections.
        if let Some(sigma) = height_sigma {
            root.fixed_view_mut::<1, 3>(8, 6)
                .copy_from(&(rotation.matrix().row(2) / sigma));
        }
        self.graph.add_factor(TrajectoryPrior {
            controls: control_keys(&self.controls, segment)?,
            duration: self.parameters.timing.trajectory_spacing.as_secs_f64(),
            tau,
            reference,
            information_root: root,
        })?;
        Ok(())
    }

    pub(crate) fn latest_time(&self) -> Time {
        self.latest_time
    }

    pub(crate) fn generation(&self) -> u64 {
        self.generation
    }

    fn segments(&self) -> Range<i64> {
        // Four consecutive controls support each segment. Growth and retirement
        // only change the ends of this contiguous range.
        let start = self
            .controls
            .first_key_value()
            .map_or(0, |(&index, _)| index + 1);
        let end = self
            .controls
            .last_key_value()
            .map_or(0, |(&index, _)| index - 1);
        start..end
    }

    pub fn set_parameters(&mut self, parameters: &Localization3dParameters) -> Result<()> {
        self.update_parameters(parameters.clone())
    }

    pub(crate) fn update_parameters(&mut self, parameters: Localization3dParameters) -> Result<()> {
        self.parameters
            .validate_update(&parameters)
            .map_err(|message| eyre!(message))?;
        self.optimizer.options.max_trials = parameters.solver.max_trials;
        self.parameters = parameters;
        Ok(())
    }
}

fn control_keys(
    controls: &BTreeMap<i64, StateKey<PoseControl<f64>>>,
    segment: i64,
) -> Result<[StateKey<PoseControl<f64>>; 4]> {
    let get = |index| {
        controls
            .get(&index)
            .copied()
            .ok_or_else(|| eyre!("missing control {index}"))
    };
    Ok([
        get(segment - 1)?,
        get(segment)?,
        get(segment + 1)?,
        get(segment + 2)?,
    ])
}
