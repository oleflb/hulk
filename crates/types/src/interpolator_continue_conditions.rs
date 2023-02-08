use std::time::{Duration, Instant};

use crate::motion_interpolator_staged::Condition;

pub struct NoopContinue;

impl Condition for NoopContinue {
    fn is_finished(&self) -> bool {
        true
    }
}

pub struct WaitContinue {
    start: Instant,
    duration: Duration,
}

impl WaitContinue {
    pub fn new(duration: Duration) -> Self {
        Self { start: Instant::now(), duration }
    }
}

impl Condition for WaitContinue{
    fn is_finished(&self) -> bool {
        self.start.elapsed() >= self.duration
    }
}
