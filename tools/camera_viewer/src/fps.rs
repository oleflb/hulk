use std::{collections::VecDeque, time::Instant};

const FPS_WINDOW: std::time::Duration = std::time::Duration::from_secs(1);

pub(crate) struct FpsMeter {
    samples: VecDeque<Instant>,
}

impl FpsMeter {
    pub(crate) fn new() -> Self {
        Self {
            samples: VecDeque::new(),
        }
    }

    pub(crate) fn tick(&mut self, now: Instant) -> f32 {
        self.samples.push_back(now);
        self.rate(now)
    }

    pub(crate) fn rate(&mut self, now: Instant) -> f32 {
        while self
            .samples
            .front()
            .is_some_and(|sample| now.duration_since(*sample) > FPS_WINDOW)
        {
            self.samples.pop_front();
        }

        let Some(first) = self.samples.front() else {
            return 0.0;
        };
        let Some(last) = self.samples.back() else {
            return 0.0;
        };
        let elapsed = last.duration_since(*first).as_secs_f32();
        if self.samples.len() < 2 || elapsed <= f32::EPSILON {
            0.0
        } else {
            (self.samples.len() - 1) as f32 / elapsed
        }
    }
}
