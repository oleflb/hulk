use eframe::egui::{
    Align2, Color32, CursorIcon, Event, FontId, Key, PointerButton, Pos2, Rect, Response, Sense,
    Stroke, StrokeKind, Ui, Vec2, Widget,
};
use eframe::epaint::TextureHandle;

use crate::{
    annotation::Annotation,
    boundingbox::{BoundingBox, Corner},
    classes::Class,
    user_toml::CONFIG,
};

const HANDLE_RADIUS: f32 = 5.0;
const HANDLE_HIT_RADIUS: f32 = 10.0;
const POINT_RADIUS: f32 = 5.0;
const POINT_HIT_RADIUS: f32 = 12.0;

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
    index: usize,
    shape: AnnotationShape,
}

impl Selection {
    pub fn index(self) -> usize {
        self.index
    }

    pub fn shape(self) -> AnnotationShape {
        self.shape
    }
}

#[derive(Debug, Default)]
pub struct CanvasState {
    selected: Option<Selection>,
    draft_box: Option<BoundingBox>,
    interaction: Interaction,
    keyboard_mode: KeyboardMode,
    zoom: f32,
    pan: Vec2,
}

impl CanvasState {
    pub fn selected(&self) -> Option<Selection> {
        self.selected
    }

    pub fn apply_class(&mut self, annotations: &mut Vec<Annotation>, class: Class) {
        if let Some(draft_box) = self.draft_box.take() {
            if class.requires_point() {
                annotations.push(Annotation::point(class, draft_box.rect.center()));
                self.selected = Some(Selection {
                    index: annotations.len() - 1,
                    shape: AnnotationShape::Point,
                });
            } else {
                self.draft_box = Some(draft_box);
            }
        }

        let Some(selection) = self.selected else {
            return;
        };
        let Some(annotation) = annotations.get_mut(selection.index) else {
            self.selected = None;
            return;
        };

        if class.requires_point() && annotation.point.is_none() {
            let previous_class = annotation.class;
            annotation.point = annotation
                .bounding_box
                .as_ref()
                .map(|bounding_box| bounding_box.rect.center());
            if !previous_class.requires_point() {
                annotation.bounding_box = None;
            }
            self.selected = Some(Selection {
                index: selection.index,
                shape: AnnotationShape::Point,
            });
        } else if !class.supports_points()
            && selection.shape == AnnotationShape::Point
            && let Some(point) = annotation.point.take()
        {
            if annotation.bounding_box.is_none() {
                annotation.bounding_box = Some(BoundingBox::new(
                    point - Vec2::splat(8.0),
                    point + Vec2::splat(8.0),
                ));
            }
            self.selected = Some(Selection {
                index: selection.index,
                shape: AnnotationShape::Box,
            });
        }

        annotation.class = class;
    }
}

#[derive(Debug, Clone, Copy, Default)]
enum Interaction {
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

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
enum KeyboardMode {
    #[default]
    None,
    DraftResize {
        corner: Corner,
    },
    MoveSelected,
    ResizeSelected {
        corner: Corner,
    },
}

pub struct BoundingBoxAnnotator<'a> {
    texture_handle: TextureHandle,
    image_size: [f32; 2],
    selected_class: &'a mut Class,
    creation_shape: CreationShape,
    annotations: &'a mut Vec<Annotation>,
    state: &'a mut CanvasState,
}

impl<'a> BoundingBoxAnnotator<'a> {
    pub fn new(
        image: TextureHandle,
        image_size: [f32; 2],
        annotations: &'a mut Vec<Annotation>,
        state: &'a mut CanvasState,
        selected_class: &'a mut Class,
        creation_shape: CreationShape,
    ) -> Self {
        Self {
            texture_handle: image,
            image_size,
            annotations,
            state,
            selected_class,
            creation_shape,
        }
    }

    fn handle_input(&mut self, ui: &Ui, response: &Response, transform: ImageTransform) {
        self.handle_view_input(ui, response);
        self.handle_class_input(ui);
        self.handle_keyboard_input(ui, response, transform);
        self.handle_pointer_input(ui, response, transform);

        if let Some(selection) = self.state.selected
            && selection.index >= self.annotations.len()
        {
            self.state.selected = None;
            self.state.keyboard_mode = KeyboardMode::None;
        }
    }

    fn handle_view_input(&mut self, ui: &Ui, response: &Response) {
        if !response.hovered() {
            return;
        }

        let (scroll_delta, ctrl_pressed, middle_drag_delta) = ui.input(|input| {
            let scroll_delta = input.events.iter().find_map(|event| match event {
                Event::MouseWheel { delta, .. } => Some(*delta),
                _ => None,
            });
            (
                scroll_delta,
                input.modifiers.ctrl,
                if input.pointer.button_down(PointerButton::Middle) {
                    input.pointer.delta()
                } else {
                    Vec2::ZERO
                },
            )
        });

        if middle_drag_delta != Vec2::ZERO {
            self.state.pan += middle_drag_delta;
        }

        let Some(scroll_delta) = scroll_delta else {
            return;
        };

        if ctrl_pressed {
            let zoom_factor = (scroll_delta.y / 240.0).exp();
            let old_zoom = self.state.zoom.max(1.0);
            self.state.zoom = (old_zoom * zoom_factor).clamp(1.0, 16.0);
            return;
        }

        if self.state.zoom > 1.01 {
            self.state.pan += scroll_delta;
        }
    }

    fn handle_class_input(&mut self, ui: &Ui) {
        let config = &CONFIG.get().unwrap().keybindings;
        let requested_class = ui.input(|input| {
            input
                .events
                .iter()
                .find_map(|event| match event {
                    Event::Key {
                        key,
                        pressed: true,
                        repeat: false,
                        ..
                    } => Class::from_key(*key),
                    _ => None,
                })
                .or_else(|| {
                    if config.next_class.is_pressed(input) {
                        Some(self.current_class().next())
                    } else if config.previous_class.is_pressed(input) {
                        Some(self.current_class().previous())
                    } else {
                        None
                    }
                })
        });

        let Some(requested_class) = requested_class else {
            return;
        };

        *self.selected_class = requested_class;
        self.state.apply_class(self.annotations, requested_class);
    }

    fn current_class(&self) -> Class {
        self.state
            .selected
            .and_then(|selection| self.annotations.get(selection.index))
            .map(|annotation| annotation.class)
            .unwrap_or(*self.selected_class)
    }

    fn handle_keyboard_input(&mut self, ui: &Ui, response: &Response, transform: ImageTransform) {
        let config = &CONFIG.get().unwrap().keybindings;

        if ui.input(|input| config.abort.is_pressed(input)) {
            if self.state.draft_box.take().is_some() {
                self.state.keyboard_mode = KeyboardMode::None;
            } else if self.state.keyboard_mode != KeyboardMode::None {
                self.state.keyboard_mode = KeyboardMode::None;
            } else {
                self.state.selected = None;
            }
            return;
        }

        if ui.input(|input| config.delete.is_pressed(input) || input.key_pressed(Key::Backspace)) {
            self.delete_selected();
            return;
        }

        if ui.input(|input| config.draw.is_pressed(input)) {
            if self.state.draft_box.is_some() {
                self.commit_draft_box();
            } else if self.creation_shape == CreationShape::Point {
                self.create_point_at_pointer_or_center(response, transform);
            } else {
                self.start_keyboard_draft(response, transform);
            }
        }

        if ui.input(|input| config.confirm.is_pressed(input)) {
            if self.state.draft_box.is_some() {
                self.commit_draft_box();
            } else {
                self.state.keyboard_mode = KeyboardMode::None;
            }
        }

        if ui.input(|input| config.edit.is_pressed(input))
            && let Some(selection) = self.state.selected
            && selection.shape == AnnotationShape::Box
            && let Some(annotation) = self.annotations.get(selection.index)
            && let Some(bounding_box) = &annotation.bounding_box
        {
            let pointer = response
                .hover_pos()
                .map(|position| transform.screen_to_image_clamped(position, self.image_size))
                .unwrap_or_else(|| bounding_box.rect.center());
            self.state.keyboard_mode = KeyboardMode::ResizeSelected {
                corner: bounding_box.closest_corner(pointer),
            };
        }

        if ui.input(|input| config.move_box.is_pressed(input)) && self.state.selected.is_some() {
            self.state.keyboard_mode = KeyboardMode::MoveSelected;
        }

        if ui.input(|input| config.cycle_corner.is_pressed(input)) {
            self.state.keyboard_mode = match self.state.keyboard_mode {
                KeyboardMode::DraftResize { corner } => KeyboardMode::DraftResize {
                    corner: corner.next(),
                },
                KeyboardMode::ResizeSelected { corner } => KeyboardMode::ResizeSelected {
                    corner: corner.next(),
                },
                other => other,
            };
        }

        self.handle_tab_selection(ui);

        if let Some(delta) = keyboard_delta(ui) {
            self.apply_keyboard_delta(delta);
        }
    }

    fn handle_tab_selection(&mut self, ui: &Ui) {
        let tab_direction = ui.input(|input| {
            if input.key_pressed(Key::Tab) {
                Some(if input.modifiers.shift { -1 } else { 1 })
            } else {
                None
            }
        });

        let Some(direction) = tab_direction else {
            return;
        };
        let selectable = selectable_shapes(self.annotations);
        if selectable.is_empty() {
            self.state.selected = None;
            return;
        }

        let current = self
            .state
            .selected
            .and_then(|selection| {
                selectable
                    .iter()
                    .position(|candidate| *candidate == selection)
            })
            .unwrap_or(0);
        let len = selectable.len();
        self.state.selected = Some(if direction > 0 {
            selectable[(current + 1) % len]
        } else {
            selectable[(current + len - 1) % len]
        });
        self.state.keyboard_mode = KeyboardMode::None;
        if let Some(selection) = self.state.selected {
            *self.selected_class = self.annotations[selection.index].class;
        }
    }

    fn apply_keyboard_delta(&mut self, delta: Vec2) {
        match self.state.keyboard_mode {
            KeyboardMode::DraftResize { corner } => {
                if let Some(draft_box) = &mut self.state.draft_box {
                    let position = draft_box.corner(corner) + delta;
                    draft_box.set_corner(corner, position);
                    draft_box.clip_to_image(self.image_size);
                }
            }
            KeyboardMode::ResizeSelected { corner } => {
                if let Some(selection) = self.state.selected
                    && selection.shape == AnnotationShape::Box
                    && let Some(bounding_box) = self
                        .annotations
                        .get_mut(selection.index)
                        .and_then(|annotation| annotation.bounding_box.as_mut())
                {
                    let position = bounding_box.corner(corner) + delta;
                    bounding_box.set_corner(corner, position);
                    bounding_box.clip_to_image(self.image_size);
                }
            }
            KeyboardMode::MoveSelected | KeyboardMode::None => {
                if let Some(selection) = self.state.selected {
                    match selection.shape {
                        AnnotationShape::Box => {
                            if let Some(bounding_box) = self
                                .annotations
                                .get_mut(selection.index)
                                .and_then(|annotation| annotation.bounding_box.as_mut())
                            {
                                bounding_box.translate(delta, self.image_size);
                            }
                        }
                        AnnotationShape::Point => {
                            if let Some(point) = self
                                .annotations
                                .get_mut(selection.index)
                                .and_then(|annotation| annotation.point.as_mut())
                            {
                                *point = clamp_point(*point + delta, self.image_size);
                            }
                        }
                    }
                }
            }
        }
    }

    fn start_keyboard_draft(&mut self, response: &Response, transform: ImageTransform) {
        if !self.selected_class.supports_boxes() {
            self.create_point_at_pointer_or_center(response, transform);
            return;
        }

        let center = response
            .hover_pos()
            .map(|position| transform.screen_to_image_clamped(position, self.image_size))
            .unwrap_or_else(|| Pos2::new(self.image_size[0] * 0.5, self.image_size[1] * 0.5));
        let half_size = Vec2::splat(16.0);
        let mut draft_box = BoundingBox::new(center - half_size, center + half_size);
        draft_box.clip_to_image(self.image_size);
        self.state.draft_box = Some(draft_box);
        self.state.selected = None;
        self.state.keyboard_mode = KeyboardMode::DraftResize {
            corner: Corner::BottomRight,
        };
    }

    fn create_point_at_pointer_or_center(
        &mut self,
        response: &Response,
        transform: ImageTransform,
    ) {
        let point = response
            .hover_pos()
            .map(|position| transform.screen_to_image_clamped(position, self.image_size))
            .unwrap_or_else(|| Pos2::new(self.image_size[0] * 0.5, self.image_size[1] * 0.5));
        self.add_point(point);
    }

    fn add_point(&mut self, point: Pos2) {
        if !self.selected_class.supports_points() {
            return;
        }

        self.annotations.push(Annotation::point(
            *self.selected_class,
            clamp_point(point, self.image_size),
        ));
        self.state.selected = Some(Selection {
            index: self.annotations.len() - 1,
            shape: AnnotationShape::Point,
        });
        self.state.keyboard_mode = KeyboardMode::MoveSelected;
    }

    fn handle_pointer_input(&mut self, ui: &Ui, response: &Response, transform: ImageTransform) {
        let pointer_position = ui.input(|input| input.pointer.interact_pos());

        if response.clicked_by(PointerButton::Secondary) {
            self.delete_at(pointer_position, transform);
            return;
        }

        if response.drag_started_by(PointerButton::Primary)
            && let Some(position) = pointer_position
        {
            self.start_pointer_interaction(position, transform);
        }

        if ui.input(|input| input.pointer.button_down(PointerButton::Primary)) {
            if let Some(position) = pointer_position {
                self.update_pointer_interaction(position, transform);
            }
        } else {
            self.finish_pointer_interaction();
        }

        if response.clicked_by(PointerButton::Primary)
            && let Some(position) = pointer_position
        {
            self.click_pointer(position, transform);
        }
    }

    fn click_pointer(&mut self, position: Pos2, transform: ImageTransform) {
        if let Some(selection) = self.hit_test_point(position, transform) {
            self.select(selection);
            return;
        }

        let Some(image_position) = transform.screen_to_image(position) else {
            return;
        };

        if let Some(selection) = self.hit_test_box(image_position) {
            self.select(selection);
            return;
        }

        if self.creation_shape == CreationShape::Point {
            self.add_point(image_position);
        }
    }

    fn start_pointer_interaction(&mut self, position: Pos2, transform: ImageTransform) {
        if let Some(selection) = self.hit_test_point(position, transform) {
            self.select(selection);
            self.state.interaction = Interaction::MovingPoint {
                index: selection.index,
            };
            return;
        }

        if let Some((index, corner)) = self.hit_test_handle(position, transform) {
            self.select(Selection {
                index,
                shape: AnnotationShape::Box,
            });
            self.state.interaction = Interaction::ResizingBox { index, corner };
            return;
        }

        let Some(image_position) = transform.screen_to_image(position) else {
            return;
        };

        if let Some(selection) = self.hit_test_box(image_position) {
            self.select(selection);
            self.state.interaction = Interaction::MovingBox {
                index: selection.index,
                last_position: image_position,
            };
            return;
        }

        if self.creation_shape == CreationShape::Box && self.selected_class.supports_boxes() {
            self.state.selected = None;
            self.state.keyboard_mode = KeyboardMode::None;
            self.state.draft_box = Some(BoundingBox::new(image_position, image_position));
            self.state.interaction = Interaction::DrawingBox {
                start: image_position,
            };
        }
    }

    fn update_pointer_interaction(&mut self, position: Pos2, transform: ImageTransform) {
        let image_position = transform.screen_to_image_clamped(position, self.image_size);
        match self.state.interaction {
            Interaction::None => {}
            Interaction::DrawingBox { start } => {
                self.state.draft_box = Some(BoundingBox::new(start, image_position));
            }
            Interaction::MovingBox {
                index,
                last_position,
            } => {
                if let Some(bounding_box) = self
                    .annotations
                    .get_mut(index)
                    .and_then(|annotation| annotation.bounding_box.as_mut())
                {
                    bounding_box.translate(image_position - last_position, self.image_size);
                    self.state.interaction = Interaction::MovingBox {
                        index,
                        last_position: image_position,
                    };
                }
            }
            Interaction::ResizingBox { index, corner } => {
                if let Some(bounding_box) = self
                    .annotations
                    .get_mut(index)
                    .and_then(|annotation| annotation.bounding_box.as_mut())
                {
                    bounding_box.set_corner(corner, image_position);
                    bounding_box.clip_to_image(self.image_size);
                }
            }
            Interaction::MovingPoint { index } => {
                if let Some(point) = self
                    .annotations
                    .get_mut(index)
                    .and_then(|annotation| annotation.point.as_mut())
                {
                    *point = image_position;
                }
            }
        }
    }

    fn finish_pointer_interaction(&mut self) {
        if matches!(self.state.interaction, Interaction::DrawingBox { .. }) {
            self.commit_draft_box();
        }
        self.state.interaction = Interaction::None;
    }

    fn delete_at(&mut self, pointer_position: Option<Pos2>, transform: ImageTransform) {
        if let Some(position) = pointer_position {
            if let Some(selection) = self.hit_test_point(position, transform) {
                self.delete_selection(selection);
                return;
            }

            if let Some(image_position) = transform.screen_to_image(position)
                && let Some(selection) = self.hit_test_box(image_position)
            {
                self.delete_selection(selection);
                return;
            }
        }

        self.delete_selected();
    }

    fn delete_selected(&mut self) {
        if let Some(selection) = self.state.selected.take() {
            self.delete_selection(selection);
        }
        self.state.keyboard_mode = KeyboardMode::None;
    }

    fn delete_selection(&mut self, selection: Selection) {
        let Some(annotation) = self.annotations.get_mut(selection.index) else {
            return;
        };
        match selection.shape {
            AnnotationShape::Box => annotation.bounding_box = None,
            AnnotationShape::Point => annotation.point = None,
        }
        if annotation.is_empty() {
            self.annotations.remove(selection.index);
        }
        self.state.selected = None;
        self.state.keyboard_mode = KeyboardMode::None;
    }

    fn commit_draft_box(&mut self) {
        let Some(mut draft_box) = self.state.draft_box.take() else {
            return;
        };
        if !self.selected_class.supports_boxes() {
            self.add_point(draft_box.rect.center());
            return;
        }
        draft_box.clip_to_image(self.image_size);
        if draft_box.is_valid() {
            self.annotations
                .push(Annotation::bounding_box(*self.selected_class, draft_box));
            self.state.selected = Some(Selection {
                index: self.annotations.len() - 1,
                shape: AnnotationShape::Box,
            });
        }
        self.state.keyboard_mode = KeyboardMode::None;
    }

    fn select(&mut self, selection: Selection) {
        self.state.selected = Some(selection);
        self.state.keyboard_mode = KeyboardMode::None;
        if let Some(annotation) = self.annotations.get(selection.index) {
            *self.selected_class = annotation.class;
        }
    }

    fn hit_test_point(
        &self,
        screen_position: Pos2,
        transform: ImageTransform,
    ) -> Option<Selection> {
        self.annotations
            .iter()
            .enumerate()
            .rev()
            .find_map(|(index, annotation)| {
                annotation.point.and_then(|point| {
                    (transform.image_to_screen(point).distance(screen_position) <= POINT_HIT_RADIUS)
                        .then_some(Selection {
                            index,
                            shape: AnnotationShape::Point,
                        })
                })
            })
    }

    fn hit_test_handle(
        &self,
        screen_position: Pos2,
        transform: ImageTransform,
    ) -> Option<(usize, Corner)> {
        self.annotations
            .iter()
            .enumerate()
            .rev()
            .filter(|(_, annotation)| annotation.class.supports_boxes())
            .find_map(|(index, annotation)| {
                annotation.bounding_box.as_ref().and_then(|bounding_box| {
                    Corner::ALL.into_iter().find_map(|corner| {
                        let corner_screen = transform.image_to_screen(bounding_box.corner(corner));
                        (corner_screen.distance(screen_position) <= HANDLE_HIT_RADIUS)
                            .then_some((index, corner))
                    })
                })
            })
    }

    fn hit_test_box(&self, image_position: Pos2) -> Option<Selection> {
        self.annotations
            .iter()
            .enumerate()
            .filter(|(_, annotation)| annotation.class.supports_boxes())
            .filter_map(|(index, annotation)| {
                annotation
                    .bounding_box
                    .as_ref()
                    .filter(|bounding_box| bounding_box.contains(image_position))
                    .map(|bounding_box| (index, bounding_box))
            })
            .min_by(|(_, left), (_, right)| left.rect.area().total_cmp(&right.rect.area()))
            .map(|(index, _)| Selection {
                index,
                shape: AnnotationShape::Box,
            })
    }

    fn draw(&self, ui: &Ui, response: &Response, transform: ImageTransform) {
        let painter = ui.painter_at(response.rect);
        painter.rect_filled(response.rect, 8.0, Color32::from_rgb(17, 17, 27));
        painter.rect_filled(transform.image_rect, 4.0, Color32::from_rgb(24, 24, 37));
        painter.image(
            self.texture_handle.id(),
            transform.image_rect,
            Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)),
            Color32::WHITE,
        );

        for (index, annotation) in self.annotations.iter().enumerate() {
            if let Some(bounding_box) = &annotation.bounding_box {
                self.draw_box(
                    ui,
                    transform,
                    annotation.class,
                    bounding_box,
                    self.state.selected
                        == Some(Selection {
                            index,
                            shape: AnnotationShape::Box,
                        }),
                    false,
                );
            }
        }

        if let Some(draft_box) = &self.state.draft_box {
            self.draw_box(ui, transform, *self.selected_class, draft_box, true, true);
        }

        for (index, annotation) in self.annotations.iter().enumerate() {
            if let Some(point) = annotation.point {
                self.draw_point(
                    ui,
                    transform,
                    annotation,
                    point,
                    self.state.selected
                        == Some(Selection {
                            index,
                            shape: AnnotationShape::Point,
                        }),
                );
            }
        }
    }

    fn draw_box(
        &self,
        ui: &Ui,
        transform: ImageTransform,
        class: Class,
        bounding_box: &BoundingBox,
        selected: bool,
        draft: bool,
    ) {
        if !bounding_box.is_valid() && !draft {
            return;
        }

        let painter = ui.painter();
        let rect = transform.image_rect_to_screen(bounding_box.rect);
        let editable = class.supports_boxes() || draft;
        let color = if editable {
            class.color()
        } else {
            class.color().gamma_multiply(0.55)
        };
        let stroke_width = if selected { 2.5 } else { 1.5 };
        painter.rect_filled(
            rect,
            2.0,
            color.gamma_multiply(if selected { 0.18 } else { 0.10 }),
        );
        painter.rect_stroke(
            rect,
            2.0,
            Stroke::new(stroke_width, color),
            StrokeKind::Inside,
        );

        let label = if draft {
            format!("new {}", class.as_str())
        } else if editable {
            class.as_str().to_string()
        } else {
            format!("legacy {} box", class.as_str())
        };
        painter.text(
            rect.left_top() + Vec2::new(4.0, 4.0),
            Align2::LEFT_TOP,
            label,
            FontId::proportional(13.0),
            Color32::from_rgb(205, 214, 244),
        );

        if editable && (selected || draft) {
            for corner in Corner::ALL {
                let corner_screen = transform.image_to_screen(bounding_box.corner(corner));
                painter.circle_filled(corner_screen, HANDLE_RADIUS, color);
                painter.circle_stroke(
                    corner_screen,
                    HANDLE_RADIUS + 1.5,
                    Stroke::new(1.0, Color32::from_rgb(205, 214, 244)),
                );
            }
        }
    }

    fn draw_point(
        &self,
        ui: &Ui,
        transform: ImageTransform,
        annotation: &Annotation,
        point: Pos2,
        selected: bool,
    ) {
        let painter = ui.painter();
        let position = transform.image_to_screen(point);
        let color = annotation.class.color();
        let radius = if selected {
            POINT_RADIUS + 2.0
        } else {
            POINT_RADIUS
        };

        painter.circle_filled(position, radius, color);
        painter.circle_stroke(
            position,
            radius + 2.0,
            Stroke::new(1.5, Color32::from_rgb(205, 214, 244)),
        );
        painter.line_segment(
            [
                position - Vec2::new(9.0, 0.0),
                position + Vec2::new(9.0, 0.0),
            ],
            Stroke::new(1.5, color),
        );
        painter.line_segment(
            [
                position - Vec2::new(0.0, 9.0),
                position + Vec2::new(0.0, 9.0),
            ],
            Stroke::new(1.5, color),
        );
        painter.text(
            position + Vec2::new(8.0, -8.0),
            Align2::LEFT_BOTTOM,
            annotation.class.as_str(),
            FontId::proportional(13.0),
            Color32::from_rgb(205, 214, 244),
        );
    }
}

impl Widget for BoundingBoxAnnotator<'_> {
    fn ui(mut self, ui: &mut Ui) -> Response {
        if self.state.zoom <= 0.0 {
            self.state.zoom = 1.0;
        }

        let available_size = ui.available_size_before_wrap();
        let desired_height = available_size.y.max(360.0);
        let desired_size = Vec2::new(available_size.x.max(360.0), desired_height);
        let (rect, mut response) = ui.allocate_exact_size(desired_size, Sense::click_and_drag());
        response.widget_info(|| eframe::egui::WidgetInfo::new(eframe::egui::WidgetType::Other));

        if response.hovered() {
            response = response.on_hover_cursor(CursorIcon::Crosshair);
        }

        let transform = ImageTransform::new(rect, self.image_size, self.state.zoom, self.state.pan);
        self.handle_input(ui, &response, transform);
        self.draw(ui, &response, transform);

        response
    }
}

#[derive(Debug, Clone, Copy)]
struct ImageTransform {
    image_rect: Rect,
    scale: f32,
}

impl ImageTransform {
    fn new(canvas_rect: Rect, image_size: [f32; 2], zoom: f32, pan: Vec2) -> Self {
        let padding = Vec2::splat(16.0);
        let available = (canvas_rect.size() - 2.0 * padding).max(Vec2::splat(1.0));
        let base_scale = (available.x / image_size[0])
            .min(available.y / image_size[1])
            .max(0.001);
        let scale = base_scale * zoom.max(1.0);
        let displayed_size = Vec2::new(image_size[0] * scale, image_size[1] * scale);
        let image_rect = Rect::from_center_size(canvas_rect.center() + pan, displayed_size);

        Self { image_rect, scale }
    }

    fn screen_to_image(self, screen_position: Pos2) -> Option<Pos2> {
        self.image_rect.contains(screen_position).then(|| {
            Pos2::new(
                (screen_position.x - self.image_rect.left()) / self.scale,
                (screen_position.y - self.image_rect.top()) / self.scale,
            )
        })
    }

    fn screen_to_image_clamped(self, screen_position: Pos2, image_size: [f32; 2]) -> Pos2 {
        Pos2::new(
            ((screen_position.x - self.image_rect.left()) / self.scale).clamp(0.0, image_size[0]),
            ((screen_position.y - self.image_rect.top()) / self.scale).clamp(0.0, image_size[1]),
        )
    }

    fn image_to_screen(self, image_position: Pos2) -> Pos2 {
        Pos2::new(
            self.image_rect.left() + image_position.x * self.scale,
            self.image_rect.top() + image_position.y * self.scale,
        )
    }

    fn image_rect_to_screen(self, image_rect: Rect) -> Rect {
        Rect::from_min_max(
            self.image_to_screen(image_rect.min),
            self.image_to_screen(image_rect.max),
        )
    }
}

fn selectable_shapes(annotations: &[Annotation]) -> Vec<Selection> {
    annotations
        .iter()
        .enumerate()
        .flat_map(|(index, annotation)| {
            let point = annotation.point.map(|_| Selection {
                index,
                shape: AnnotationShape::Point,
            });
            let bounding_box = (annotation.bounding_box.is_some()
                && annotation.class.supports_boxes())
            .then_some(Selection {
                index,
                shape: AnnotationShape::Box,
            });
            [point, bounding_box].into_iter().flatten()
        })
        .collect()
}

fn clamp_point(point: Pos2, image_size: [f32; 2]) -> Pos2 {
    Pos2::new(
        point.x.clamp(0.0, image_size[0]),
        point.y.clamp(0.0, image_size[1]),
    )
}

fn keyboard_delta(ui: &Ui) -> Option<Vec2> {
    ui.input(|input| {
        let step = if input.modifiers.ctrl {
            1.0
        } else if input.modifiers.shift {
            20.0
        } else {
            5.0
        };
        let mut delta = Vec2::ZERO;
        if input.key_pressed(Key::ArrowLeft) {
            delta.x -= step;
        }
        if input.key_pressed(Key::ArrowRight) {
            delta.x += step;
        }
        if input.key_pressed(Key::ArrowUp) {
            delta.y -= step;
        }
        if input.key_pressed(Key::ArrowDown) {
            delta.y += step;
        }

        (delta != Vec2::ZERO).then_some(delta)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn affected_classes_do_not_offer_bbox_selection() {
        let annotations = vec![Annotation::bounding_box(
            Class::LSpot,
            BoundingBox::new(Pos2::ZERO, Pos2::new(10.0, 10.0)),
        )];

        assert!(selectable_shapes(&annotations).is_empty());
    }

    #[test]
    fn reclassifying_regular_box_to_point_class_removes_box() {
        let mut annotations = vec![Annotation::bounding_box(
            Class::Robot,
            BoundingBox::new(Pos2::ZERO, Pos2::new(10.0, 10.0)),
        )];
        let mut state = CanvasState {
            selected: Some(Selection {
                index: 0,
                shape: AnnotationShape::Box,
            }),
            ..Default::default()
        };

        state.apply_class(&mut annotations, Class::LSpot);

        assert_eq!(annotations[0].class, Class::LSpot);
        assert_eq!(annotations[0].point, Some(Pos2::new(5.0, 5.0)));
        assert!(annotations[0].bounding_box.is_none());
        assert_eq!(state.selected.unwrap().shape, AnnotationShape::Point);
    }

    #[test]
    fn reclassifying_legacy_point_class_box_preserves_box() {
        let mut annotations = vec![Annotation::bounding_box(
            Class::LSpot,
            BoundingBox::new(Pos2::ZERO, Pos2::new(10.0, 10.0)),
        )];
        let mut state = CanvasState {
            selected: Some(Selection {
                index: 0,
                shape: AnnotationShape::Box,
            }),
            ..Default::default()
        };

        state.apply_class(&mut annotations, Class::TSpot);

        assert_eq!(annotations[0].class, Class::TSpot);
        assert_eq!(annotations[0].point, Some(Pos2::new(5.0, 5.0)));
        assert!(annotations[0].bounding_box.is_some());
        assert_eq!(state.selected.unwrap().shape, AnnotationShape::Point);
    }
}
