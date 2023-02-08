#![allow(dead_code)]

use crate::{
    interpolator_continue_conditions::{NoopContinue, WaitContinue},
    Joints, MotionFile, MotionFileFrame, MotionFileInterpolator, LinearInterpolator,
};
use std::time::{Duration, Instant};

pub trait Condition: Send + Sync {
    fn is_finished(&self) -> bool;
}

pub struct ConditionMotionFileInterpolator {
    interpolator: LinearInterpolator<Joints>,
    condition: Box<dyn Condition>,
}

impl ConditionMotionFileInterpolator {
    pub fn new(
        interpolator: LinearInterpolator<Joints>,
        condition: Box<dyn Condition>,
    ) -> Self {
        Self {
            interpolator,
            condition,
        }
    }

    pub fn step(&mut self, time_step: Duration) -> Joints {
        self.interpolator.step(time_step)
    }

    pub fn value(&self) -> Joints {
        self.interpolator.value()
    }

    pub fn is_finished(&self) -> bool {
        self.interpolator.is_finished() && self.condition.is_finished()
    }

    pub fn reset(&mut self) {
        self.interpolator.reset();
    }
}

pub struct StagedMotionFileInterpolator {
    stages: Vec<ConditionMotionFileInterpolator>,
    current_stage: usize,
    interpolator_start_time: Instant,
    stage_end_time: Instant,
}

impl StagedMotionFileInterpolator {
    pub fn new(stages: Vec<ConditionMotionFileInterpolator>) -> Self {
        assert!(
            !stages.is_empty(),
            "empty stages given to staged interpolator"
        );

        StagedMotionFileInterpolator {
            stages,
            current_stage: 0,
            interpolator_start_time: Instant::now(),
            stage_end_time: Instant::now(),
        }
    }

    pub fn from_motion_file_interpolator(motion_file: MotionFileInterpolator) -> Self {
        todo!();
        // let MotionFileInterpolator{ interpolators, interpolator_index: _ } = motion_file;

        // let mut staged_interpolators = vec![interpolators[0]];

        // for interpolator in interpolators[1..].iter() {
        //     let mut last_interpolator = staged_interpolators.last_mut().unwrap();
        //     if last_interpolator.end() == interpolator.start() {
        //         // Can compact both interpolators
                
        //     }
        // }

        // todo!()
    }

    pub fn reset(&mut self) {
        self.stages
            .iter_mut()
            .for_each(|interpolator| interpolator.reset());
        self.current_stage = 0;
    }

    pub fn step(&mut self, time_step: Duration) -> Joints {
        while self.stages[self.current_stage].is_finished()
            && self.current_stage < self.stages.len() - 1
        {
            self.current_stage += 1;
        }

        self.stages[self.current_stage].step(time_step)
    }

    pub fn is_finished(&self) -> bool {
        self.stages.last().unwrap().is_finished()
    }
}
