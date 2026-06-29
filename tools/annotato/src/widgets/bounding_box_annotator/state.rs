use eframe::egui::{Pos2, Vec2};

use crate::boundingbox::{BoundingBox, Corner};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CreationShape {
    Box,
    Point,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AnnotationShape {
    Box,
    Point,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Selection {
    pub(super) index: usize,
    pub(super) shape: AnnotationShape,
}

impl Selection {
    pub fn index(self) -> usize {
        self.index
    }

    pub fn shape(self) -> AnnotationShape {
        self.shape
    }
}

#[derive(Debug)]
pub struct CanvasState {
    pub(super) selected: Option<Selection>,
    pub(super) mode: CanvasMode,
    pub(super) focus_anchor: Option<FocusAnchor>,
    pub(super) zoom: f32,
    pub(super) pan: Vec2,
    annotations_changed: bool,
}

impl Default for CanvasState {
    fn default() -> Self {
        Self {
            selected: None,
            mode: CanvasMode::Idle,
            focus_anchor: None,
            zoom: 1.0,
            pan: Vec2::ZERO,
            annotations_changed: false,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(super) struct FocusAnchor {
    pub(super) screen_position: Pos2,
    pub(super) image_position: Pos2,
}

impl CanvasState {
    pub fn selected(&self) -> Option<Selection> {
        self.selected
    }

    pub(super) fn clear_mode(&mut self) {
        self.mode = CanvasMode::Idle;
    }

    pub(super) fn draft_box(&self) -> Option<BoundingBox> {
        match self.mode {
            CanvasMode::DrawingBox { start, moving }
            | CanvasMode::KeyboardDraftBox {
                anchor: start,
                moving,
                ..
            } => Some(BoundingBox::new(start, moving)),
            _ => None,
        }
    }

    pub(super) fn take_draft_box(&mut self) -> Option<BoundingBox> {
        let draft_box = self.draft_box();
        if draft_box.is_some() {
            self.clear_mode();
        }
        draft_box
    }

    pub fn mark_annotations_changed(&mut self) {
        self.annotations_changed = true;
    }

    pub fn take_annotations_changed(&mut self) -> bool {
        let annotations_changed = self.annotations_changed;
        self.annotations_changed = false;
        annotations_changed
    }

    pub fn pointer_interaction_active(&self) -> bool {
        self.mode.is_pointer_interaction()
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(super) enum CanvasMode {
    #[default]
    Idle,
    DrawingBox {
        start: Pos2,
        moving: Pos2,
    },
    DraggingBox {
        index: usize,
        last_position: Pos2,
    },
    ResizingBoxWithPointer {
        index: usize,
        corner: Corner,
    },
    DraggingPoint {
        index: usize,
    },
    KeyboardDraftBox {
        anchor: Pos2,
        moving: Pos2,
        pointer_position: Option<Pos2>,
    },
    KeyboardMoveSelected,
    KeyboardResizeSelected {
        corner: Corner,
    },
}

impl CanvasMode {
    pub(super) fn is_idle(self) -> bool {
        matches!(self, Self::Idle)
    }

    pub(super) fn is_pointer_interaction(self) -> bool {
        matches!(
            self,
            Self::DraggingBox { .. }
                | Self::ResizingBoxWithPointer { .. }
                | Self::DraggingPoint { .. }
        )
    }
}
