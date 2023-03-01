#![allow(dead_code)]

use splines::{Interpolation, Key, Spline};

use crate::{
    interpolator_continue_conditions::{NoopContinue},
    Joints, MotionFile, MotionFileInterpolator,
};
use std::{
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

impl From<MotionFile> for ConditionMotionFileInterpolator {
    fn from(motion_file: MotionFile) -> Self {
        assert!(!motion_file.frames.is_empty());

        let mut current_time = Duration::ZERO;
        let mut keys = vec![Key::new(current_time, motion_file.initial_positions, Interpolation::Linear)];

        keys.extend(motion_file.frames.into_iter().map(|frame| {
            current_time += frame.duration;
            Key::new(current_time, frame.positions, Interpolation::Linear)
        }));

        Self::new(keys, Box::new(NoopContinue {}))
    }
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
            2. * keys[0].t.as_secs_f32() - keys[1].t.as_secs_f32(),
            keys[1].value,
            Interpolation::CatmullRom,
        );
        let right_helper = Key::new(
            2. * keys[last_key_index].t.as_secs_f32() - keys[last_key_index - 1].t.as_secs_f32(),
            keys[last_key_index - 1].value,
            Interpolation::CatmullRom,
        );
        
        let mut interpolator = Spline::from_iter(
            keys.into_iter()
                .map(|key| Key::new(key.t.as_secs_f32(), key.value, Interpolation::CatmullRom)),
        );
        interpolator.add(left_helper);
        interpolator.add(right_helper);

        println!("Start time is {:?}, End time is {:?}", start_time, end_time);
        
        Self {
            interpolator,
            condition,
            current_time,
            start_time,
            end_time,
        }
    }

    pub fn step(&mut self, time_step: Duration) -> Joints {
        self.current_time += time_step;
        self.value()
    }
    
    pub fn value(&self) -> Joints {
        if self.current_time <= self.start_time {
            self.interpolator.sample(self.start_time.as_secs_f32())
        } else if self.current_time >= self.end_time {
            self.interpolator.sample(self.end_time.as_secs_f32())
        } else {
            self.interpolator.sample(self.current_time.as_secs_f32())
        }.unwrap_or_else(|| panic!("the interpolator was sampled at {}s where no key is present", self.current_time.as_secs_f32()))
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
