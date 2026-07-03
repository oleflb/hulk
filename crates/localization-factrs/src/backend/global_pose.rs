use factrs::{containers::FactorBuilder, core::Huber, traits::Optimizer};

use crate::{
    factors::global_pose::GlobalPoseFactor, measurements::GlobalPoseMeasurement, symbols::State,
};

use super::VinsBackend;

impl VinsBackend {
    pub(super) fn ingest_global_poses(&mut self, poses: Vec<GlobalPoseMeasurement>) {
        let Some(last) = poses.last() else {
            return;
        };

        self.update_last_knot_time(last.time);
        let interval_groups = self.interval_groups(poses, |pose| pose.time);

        for group in interval_groups {
            if !self.prepare_interval_for_measurements(group.start_index, "global pose") {
                continue;
            }

            let keys = (State(group.start_index), State(group.start_index + 1));
            let graph = self.optimizer.graph_mut();
            for measurement in group.measurements {
                let residual = GlobalPoseFactor::new(
                    group.start_time,
                    group.end_time,
                    [measurement],
                    self.config.global_pose_noise,
                );
                let factor = FactorBuilder::new(residual, keys)
                    .robust(Huber::new(self.config.global_pose_huber_threshold))
                    .build();

                graph.add_factor(factor);
            }
        }
    }
}
