use std::ops::Range;

use crate::classes::Class;

pub const CHUNK_SIZE: usize = 50;

#[derive(Debug, Clone, Copy, Default)]
pub struct ChunkWorkflow {
    class_index: usize,
}

impl ChunkWorkflow {
    pub fn active_class(self) -> Class {
        Class::ALL[self.class_index]
    }

    pub fn apply_position(&mut self, position: ChunkPosition) {
        self.class_index = position.class_index;
    }

    pub fn is_active_class(self, position: ChunkPosition) -> bool {
        self.class_index == position.class_index
    }

    pub fn first_pending_position<F>(
        image_count: usize,
        mut is_complete: F,
    ) -> Option<ChunkPosition>
    where
        F: FnMut(usize, Class) -> bool,
    {
        let mut chunk_start = 0;
        while chunk_start < image_count {
            let chunk_range = chunk_range_from_start(chunk_start, image_count);
            for class_index in 0..Class::ALL.len() {
                if let Some(index) = first_pending_in_range(
                    chunk_range.clone(),
                    image_count,
                    class_index,
                    &mut is_complete,
                ) {
                    return Some(ChunkPosition::new(index, class_index));
                }
            }
            chunk_start = chunk_range.end;
        }
        None
    }

    pub fn next_pending_position_after<F>(
        self,
        current_index: usize,
        image_count: usize,
        mut is_complete: F,
    ) -> Option<ChunkPosition>
    where
        F: FnMut(usize, Class) -> bool,
    {
        let class_index = self.class_index;
        let chunk_range = chunk_range_for_index(current_index, image_count);

        if let Some(index) = first_pending_in_range(
            current_index + 1..chunk_range.end,
            image_count,
            class_index,
            &mut is_complete,
        ) {
            return Some(ChunkPosition::new(index, class_index));
        }

        for next_class_index in class_index + 1..Class::ALL.len() {
            if let Some(index) = first_pending_in_range(
                chunk_range.clone(),
                image_count,
                next_class_index,
                &mut is_complete,
            ) {
                return Some(ChunkPosition::new(index, next_class_index));
            }
        }

        let mut next_chunk_start = chunk_range.end;
        while next_chunk_start < image_count {
            let next_chunk_range = chunk_range_from_start(next_chunk_start, image_count);
            for next_class_index in 0..Class::ALL.len() {
                if let Some(index) = first_pending_in_range(
                    next_chunk_range.clone(),
                    image_count,
                    next_class_index,
                    &mut is_complete,
                ) {
                    return Some(ChunkPosition::new(index, next_class_index));
                }
            }
            next_chunk_start = next_chunk_range.end;
        }

        None
    }

    pub fn previous_position_before(
        self,
        current_index: usize,
        image_count: usize,
    ) -> Option<ChunkPosition> {
        let chunk_start = chunk_start(current_index);
        let chunk_range = chunk_range_from_start(chunk_start, image_count);
        let class_index = self.class_index;

        if current_index > chunk_start {
            return Some(ChunkPosition::new(current_index - 1, class_index));
        }

        if class_index > 0 {
            return Some(ChunkPosition::new(chunk_range.end - 1, class_index - 1));
        }

        if chunk_start > 0 {
            let previous_chunk_start = chunk_start.saturating_sub(CHUNK_SIZE);
            let previous_chunk_range = chunk_range_from_start(previous_chunk_start, image_count);
            return Some(ChunkPosition::new(
                previous_chunk_range.end - 1,
                Class::ALL.len() - 1,
            ));
        }

        None
    }

    pub fn chunk_progress<F>(
        self,
        current_index: usize,
        image_count: usize,
        mut is_complete: F,
    ) -> ChunkProgress
    where
        F: FnMut(usize, Class) -> bool,
    {
        let chunk_range = chunk_range_for_index(current_index, image_count);
        let active_class = self.active_class();
        let completed = chunk_range
            .clone()
            .filter(|index| is_complete(*index, active_class))
            .count();

        ChunkProgress {
            chunk_index: chunk_range.start / CHUNK_SIZE,
            chunk_count: image_count.div_ceil(CHUNK_SIZE),
            class: active_class,
            completed,
            total: chunk_range.len(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ChunkPosition {
    pub index: usize,
    class_index: usize,
}

impl ChunkPosition {
    pub fn new(index: usize, class_index: usize) -> Self {
        Self { index, class_index }
    }

    pub fn class(self) -> Class {
        Class::ALL[self.class_index]
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ClassTransition {
    pub position: ChunkPosition,
    pub direction: ClassTransitionDirection,
}

impl ClassTransition {
    pub fn new(position: ChunkPosition, direction: ClassTransitionDirection) -> Self {
        Self {
            position,
            direction,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClassTransitionDirection {
    Next,
    Previous,
}

impl ClassTransitionDirection {
    pub fn label(self) -> &'static str {
        match self {
            Self::Next => "Next class",
            Self::Previous => "Previous class",
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ChunkProgress {
    chunk_index: usize,
    chunk_count: usize,
    class: Class,
    completed: usize,
    total: usize,
}

impl ChunkProgress {
    pub fn fraction(self) -> f32 {
        self.completed as f32 / self.total.max(1) as f32
    }

    pub fn label(self) -> String {
        format!(
            "Chunk {} of {} · {} {}/{}",
            self.chunk_index + 1,
            self.chunk_count,
            self.class.as_str(),
            self.completed,
            self.total
        )
    }
}

fn first_pending_in_range<F>(
    range: Range<usize>,
    image_count: usize,
    class_index: usize,
    is_complete: &mut F,
) -> Option<usize>
where
    F: FnMut(usize, Class) -> bool,
{
    let class = Class::ALL[class_index];
    range
        .filter(|index| *index < image_count)
        .find(|index| !is_complete(*index, class))
}

fn chunk_range_for_index(index: usize, image_count: usize) -> Range<usize> {
    chunk_range_from_start(chunk_start(index), image_count)
}

fn chunk_range_from_start(start: usize, image_count: usize) -> Range<usize> {
    start..(start + CHUNK_SIZE).min(image_count)
}

fn chunk_start(index: usize) -> usize {
    index / CHUNK_SIZE * CHUNK_SIZE
}
