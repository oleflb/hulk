use std::time::Duration;

use ros_z::Message;
use serde::{Deserialize, Serialize};

use crate::global_association::GlobalAssociationConfig as GlobalLocalizerParameters;

#[derive(Clone, Debug, Deserialize, Serialize, Message)]
#[serde(default, deny_unknown_fields)]
pub struct FieldMarkAssociationParameters {
    /// Maximum separation of exposure-bracketing attitude samples.
    pub max_imu_gap: Duration,
    /// Maximum separation of exposure-bracketing camera geometry samples.
    pub max_camera_gap: Duration,
    /// Allocated at startup; changing these requires restarting the node.
    pub capacities: AssociationCapacities,
    /// Shared projection uncertainty/search budgets and global geometric gates.
    pub global_localizer: GlobalLocalizerParameters,
    pub tracking: TrackingAssociationParameters,
    /// Maximum node-side timestamp difference for association geometry.
    ///
    /// The direct API expects geometry already sampled at the detection time.
    pub max_pose_hint_age: Duration,
}

impl Default for FieldMarkAssociationParameters {
    fn default() -> Self {
        Self {
            max_imu_gap: Duration::from_millis(20),
            max_camera_gap: Duration::from_millis(20),
            capacities: AssociationCapacities::default(),
            global_localizer: GlobalLocalizerParameters::default(),
            tracking: TrackingAssociationParameters::default(),
            max_pose_hint_age: Duration::from_millis(250),
        }
    }
}

impl FieldMarkAssociationParameters {
    pub fn validate(&self) -> std::result::Result<(), String> {
        self.global_localizer.validate()?;
        self.tracking.validate()?;
        self.capacities.validate()?;
        if self.max_pose_hint_age.is_zero()
            || self.max_imu_gap.is_zero()
            || self.max_camera_gap.is_zero()
        {
            return Err("association timing limits must be > 0".to_string());
        }
        Ok(())
    }

    pub(crate) fn validate_update(&self, capacities: AssociationCapacities) -> Result<(), String> {
        self.validate()?;
        if self.capacities != capacities {
            return Err("association capacities changed: restart required".into());
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize, Message)]
#[serde(default, deny_unknown_fields)]
pub struct AssociationCapacities {
    pub camera_geometry: usize,
    pub field_dimensions: usize,
    pub estimates: usize,
    pub status: usize,
    pub attitudes: usize,
    pub imu_queue: usize,
    pub detections_queue: usize,
}

impl Default for AssociationCapacities {
    fn default() -> Self {
        Self {
            camera_geometry: 1500,
            field_dimensions: 1,
            estimates: 128,
            status: 1,
            attitudes: 1500,
            imu_queue: 500,
            detections_queue: 1,
        }
    }
}

impl AssociationCapacities {
    fn validate(self) -> Result<(), String> {
        if [
            self.camera_geometry,
            self.field_dimensions,
            self.estimates,
            self.status,
            self.attitudes,
            self.imu_queue,
            self.detections_queue,
        ]
        .contains(&0)
        {
            return Err("association capacities must be > 0".into());
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, Message)]
#[serde(default, deny_unknown_fields)]
pub struct TrackingAssociationParameters {
    /// Bounded covariance eigensolver iterations; zero (unbounded) is rejected.
    pub covariance_eigen_max_iterations: usize,
    /// Pixel residual ceiling for winning edges. Plausible rivals beyond it still enter scoring.
    pub max_pixel_distance: f32,
    /// Additional position sigma per second without a successful solve.
    pub position_sigma_per_second: f32,
    /// Additional angular sigma per second without a successful solve.
    pub yaw_sigma_per_second: f32,
    /// Validity horizon since the last successful solve; older predictions reject without fallback.
    pub max_age: Duration,
    /// For 3..=5 features, required best/rival joint Gaussian likelihood ratio;
    /// the log-likelihood gap must exceed ln(score_ratio).
    /// For >5 features, marginal-assignment heuristic: removing any winning edge must lose
    /// more than 1 - 1/score_ratio of that edge's normalized Gaussian benefit.
    pub score_ratio: f32,
}

impl Default for TrackingAssociationParameters {
    fn default() -> Self {
        Self {
            covariance_eigen_max_iterations: 64,
            max_pixel_distance: 80.0,
            position_sigma_per_second: 0.15,
            yaw_sigma_per_second: 0.1,
            max_age: Duration::from_secs(5),
            score_ratio: 1.05,
        }
    }
}

impl TrackingAssociationParameters {
    fn validate(&self) -> Result<(), String> {
        for value in [
            self.max_pixel_distance,
            self.position_sigma_per_second,
            self.yaw_sigma_per_second,
        ] {
            if !value.is_finite() || value <= 0.0 {
                return Err(
                    "tracking distance and uncertainty rates must be finite and > 0".into(),
                );
            }
        }
        if self.covariance_eigen_max_iterations == 0
            || self.max_age.is_zero()
            || !self.score_ratio.is_finite()
            || self.score_ratio <= 1.0
        {
            return Err("invalid tracking eigensolver iteration limit, age or score ratio".into());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn guided_consensus_and_height_limits_are_validated() {
        for fraction in [0.0, -0.1, 1.1, f32::NAN] {
            let config = GlobalLocalizerParameters {
                min_inlier_fraction: fraction,
                ..Default::default()
            };
            assert!(config.validate().is_err());
        }
        for ratio in [1.0, f32::NAN, f32::INFINITY] {
            let config = GlobalLocalizerParameters {
                score_ratio: ratio,
                ..Default::default()
            };
            assert!(config.validate().is_err());
        }
        let mut config = GlobalLocalizerParameters::default();
        config.max_camera_height = config.min_camera_height;
        assert!(config.validate().is_err());
        config.max_camera_height = 2.0;
        config.class_mismatch_penalty = f32::NAN;
        assert!(config.validate().is_err());
    }

    #[test]
    fn defaults_and_detection_limits_remain_compatible() {
        let parameters: FieldMarkAssociationParameters = serde_json::from_str("{}").unwrap();
        parameters.validate().unwrap();
        assert_eq!(parameters.global_localizer.max_retained_detections, 32);
        let serialized = serde_json::json!(&parameters);
        for setting in [
            "symmetry_epsilon",
            "min_pair_distance",
            "min_triangle_denominator",
        ] {
            assert!(serialized["global_localizer"].get(setting).is_none());
            let json = serde_json::json!({"global_localizer": {(setting): 0.001}});
            assert!(serde_json::from_value::<FieldMarkAssociationParameters>(json).is_err());
        }
        for setting in ["covariance_symmetry_tolerance", "covariance_psd_tolerance"] {
            assert!(serialized["tracking"].get(setting).is_none());
            let json = serde_json::json!({"tracking": {(setting): 0.001}});
            assert!(serde_json::from_value::<FieldMarkAssociationParameters>(json).is_err());
        }
        let mut config = parameters.global_localizer;
        config.max_retained_detections = 64;
        config.max_work = 2_000_000;
        config.validate().unwrap();
        config.min_inliers = 2;
        assert!(config.validate().is_err());
        config.min_inliers = 3;
        config.max_input_detections = 63;
        assert!(config.validate().is_err());
    }

    #[test]
    fn capacity_updates_require_restart_but_live_settings_do_not() {
        let mut config = FieldMarkAssociationParameters::default();
        config.capacities.imu_queue = 17;
        let capacities = config.capacities;
        assert_eq!(capacities.imu_queue, 17);
        config.validate_update(capacities).unwrap();
        for change in 0..7 {
            config.capacities = capacities;
            let capacity = match change {
                0 => &mut config.capacities.camera_geometry,
                1 => &mut config.capacities.field_dimensions,
                2 => &mut config.capacities.estimates,
                3 => &mut config.capacities.status,
                4 => &mut config.capacities.attitudes,
                5 => &mut config.capacities.imu_queue,
                _ => &mut config.capacities.detections_queue,
            };
            *capacity += 1;
            assert!(
                config
                    .validate_update(capacities)
                    .unwrap_err()
                    .contains("restart required")
            );
        }
        config.capacities = capacities;
        config.max_imu_gap = Duration::from_millis(37);
        config.global_localizer.detection_pixel_sigma = 3.0;
        config.validate_update(capacities).unwrap();
        config.capacities.imu_queue = 0;
        assert!(config.validate().is_err());
    }
}
