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
    pub(super) draft_box: Option<BoundingBox>,
    pub(super) interaction: Interaction,
    pub(super) keyboard_mode: KeyboardMode,
    pub(super) focus_anchor: Option<FocusAnchor>,
    pub(super) zoom: f32,
    pub(super) pan: Vec2,
    annotations_changed: bool,
}

impl Default for CanvasState {
    fn default() -> Self {
        Self {
            selected: None,
            draft_box: None,
            interaction: Interaction::None,
            keyboard_mode: KeyboardMode::None,
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

    pub fn mark_annotations_changed(&mut self) {
        self.annotations_changed = true;
    }

    pub fn take_annotations_changed(&mut self) -> bool {
        let annotations_changed = self.annotations_changed;
        self.annotations_changed = false;
        annotations_changed
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub(super) enum Interaction {
    #[default]
    None,
    DrawingBox {
        start: Pos2,
    },
    MovingBox {
        index: usize,
        last_position: Pos2,
    },
    ResizingBox {
        index: usize,
        corner: Corner,
    },
    MovingPoint {
        index: usize,
    },
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(super) enum KeyboardMode {
    #[default]
    None,
    DraftResize {
        anchor: Pos2,
        moving: Pos2,
        pointer_position: Option<Pos2>,
    },
    MoveSelected,
    ResizeSelected {
        corner: Corner,
    },
}
