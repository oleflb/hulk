#![allow(dead_code)]

use image::imageops::FilterType::CatmullRom;
use splines::{interpolate::Interpolator, Interpolation, Key, Spline};

use crate::{
    interpolator_continue_conditions::{NoopContinue, WaitContinue},
    Joints, LinearInterpolator, MotionFile, MotionFileFrame, MotionFileInterpolator,
};
use std::{
    ops::AddAssign,
    time::{Duration, Instant},
};

pub trait Condition: Send + Sync {
    fn is_finished(&self) -> bool;
}

pub struct ConditionMotionFileInterpolator {
    interpolator: Spline<f32, Joints>,
    condition: Box<dyn Condition>,
    current_time: Duration,
    start_time: Duration,
    end_time: Duration,
}

impl ConditionMotionFileInterpolator {
    pub fn new(keys: Vec<Key<Duration, Joints>>, condition: Box<dyn Condition>) -> Self {
        assert!(keys.len() >= 2, "need at least two keys to interpolate");
        //NOTE: assume keys are sorted
        let last_key_index = keys.len() - 1;
        
        let start_time = keys[0].t;
        let current_time = start_time;
        let end_time = keys[last_key_index].t;

        let left_helper = Key::new(
            (2 * keys[0].t - keys[1].t).as_secs_f32(),
            keys[1].value,
            Interpolation::CatmullRom,
        );
        let right_helper = Key::new(
            (2 * keys[last_key_index].t - keys[last_key_index - 1].t).as_secs_f32(),
            keys[last_key_index - 1].value,
            Interpolation::CatmullRom,
        );
        
        let mut interpolator = Spline::from_iter(
            keys.into_iter()
                .map(|key| Key::new(key.t.as_secs_f32(), key.value, Interpolation::CatmullRom)),
        );
        interpolator.add(left_helper);
        interpolator.add(right_helper);
        
        Self {
            interpolator,
            condition,
            current_time,
            start_time,
            end_time,
        }
    }

    pub fn step(&mut self, time_step: Duration) -> Joints {
        self.current_time.add_assign(time_step);
        self.value()
    }
    
    pub fn value(&self) -> Joints {
        if self.current_time <= self.start_time {
            self.interpolator.sample(self.start_time.as_secs_f32())
        } else if self.current_time >= self.end_time {
            self.interpolator.sample(self.end_time.as_secs_f32())
        } else {
            self.interpolator.sample(self.current_time.as_secs_f32())
        }.expect("the interpolator was sampled at a time where no key is present")
    }

    pub fn is_finished(&self) -> bool {
        self.current_time >= self.end_time && self.condition.is_finished()
    }

    pub fn reset(&mut self) {
        self.current_time = self.start_time;
        //TODO: may add condition.reset()
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
