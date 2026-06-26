use eframe::egui::{
    Align2, Color32, CursorIcon, Event, FontId, Key, PointerButton, Pos2, Rect, Response, Sense,
    Stroke, StrokeKind, Ui, Vec2, Widget,
};
use eframe::epaint::TextureHandle;

use crate::{
    boundingbox::{BoundingBox, Corner},
    classes::Class,
    user_toml::CONFIG,
};

const HANDLE_RADIUS: f32 = 5.0;
const HANDLE_HIT_RADIUS: f32 = 10.0;

#[derive(Debug, Default)]
pub struct CanvasState {
    selected_box: Option<usize>,
    draft_box: Option<BoundingBox>,
    interaction: Interaction,
    keyboard_mode: KeyboardMode,
    zoom: f32,
    pan: Vec2,
}

impl CanvasState {
    pub fn selected_box(&self) -> Option<usize> {
        self.selected_box
    }

    pub fn apply_class(&mut self, bounding_boxes: &mut [BoundingBox], class: Class) {
        if let Some(draft_box) = &mut self.draft_box {
            draft_box.class = class;
        }
        if let Some(index) = self.selected_box
            && let Some(bounding_box) = bounding_boxes.get_mut(index)
        {
            bounding_box.class = class;
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
enum Interaction {
    #[default]
    None,
    Drawing {
        start: Pos2,
    },
    Moving {
        index: usize,
        last_position: Pos2,
    },
    Resizing {
        index: usize,
        corner: Corner,
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
    bounding_boxes: &'a mut Vec<BoundingBox>,
    state: &'a mut CanvasState,
}

impl<'a> BoundingBoxAnnotator<'a> {
    pub fn new(
        image: TextureHandle,
        image_size: [f32; 2],
        bounding_boxes: &'a mut Vec<BoundingBox>,
        state: &'a mut CanvasState,
        selected_class: &'a mut Class,
    ) -> Self {
        Self {
            texture_handle: image,
            image_size,
            bounding_boxes,
            state,
            selected_class,
        }
    }

    fn handle_input(&mut self, ui: &Ui, response: &Response, transform: ImageTransform) {
        self.handle_view_input(ui, response);
        self.handle_class_input(ui);
        self.handle_keyboard_box_input(ui, response, transform);
        self.handle_pointer_input(ui, response, transform);

        if let Some(selected_box) = self.state.selected_box
            && selected_box >= self.bounding_boxes.len()
        {
            self.state.selected_box = None;
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
        if let Some(draft_box) = &mut self.state.draft_box {
            draft_box.class = requested_class;
        }
        if let Some(index) = self.state.selected_box
            && let Some(bounding_box) = self.bounding_boxes.get_mut(index)
        {
            bounding_box.class = requested_class;
        }
    }

    fn current_class(&self) -> Class {
        if let Some(draft_box) = &self.state.draft_box {
            return draft_box.class;
        }
        self.state
            .selected_box
            .and_then(|index| self.bounding_boxes.get(index))
            .map(|bounding_box| bounding_box.class)
            .unwrap_or(*self.selected_class)
    }

    fn handle_keyboard_box_input(
        &mut self,
        ui: &Ui,
        response: &Response,
        transform: ImageTransform,
    ) {
        let config = &CONFIG.get().unwrap().keybindings;

        if ui.input(|input| config.abort.is_pressed(input)) {
            if self.state.draft_box.take().is_some() {
                self.state.keyboard_mode = KeyboardMode::None;
            } else if self.state.keyboard_mode != KeyboardMode::None {
                self.state.keyboard_mode = KeyboardMode::None;
            } else {
                self.state.selected_box = None;
            }
            return;
        }

        if ui.input(|input| config.delete.is_pressed(input) || input.key_pressed(Key::Backspace)) {
            self.delete_selected_box();
            return;
        }

        if ui.input(|input| config.draw.is_pressed(input)) {
            if self.state.draft_box.is_some() {
                self.commit_draft_box();
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
            && let Some(index) = self.state.selected_box
            && let Some(bounding_box) = self.bounding_boxes.get(index)
        {
            let pointer = response
                .hover_pos()
                .map(|position| transform.screen_to_image_clamped(position, self.image_size))
                .unwrap_or_else(|| bounding_box.rect.center());
            self.state.keyboard_mode = KeyboardMode::ResizeSelected {
                corner: bounding_box.closest_corner(pointer),
            };
        }

        if ui.input(|input| config.move_box.is_pressed(input)) && self.state.selected_box.is_some()
        {
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
        if self.bounding_boxes.is_empty() {
            self.state.selected_box = None;
            return;
        }

        let current = self.state.selected_box.unwrap_or(0);
        let len = self.bounding_boxes.len();
        self.state.selected_box = Some(if direction > 0 {
            (current + 1) % len
        } else {
            (current + len - 1) % len
        });
        self.state.keyboard_mode = KeyboardMode::None;
        if let Some(index) = self.state.selected_box {
            *self.selected_class = self.bounding_boxes[index].class;
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
                if let Some(index) = self.state.selected_box
                    && let Some(bounding_box) = self.bounding_boxes.get_mut(index)
                {
                    let position = bounding_box.corner(corner) + delta;
                    bounding_box.set_corner(corner, position);
                    bounding_box.clip_to_image(self.image_size);
                }
            }
            KeyboardMode::MoveSelected | KeyboardMode::None => {
                if let Some(index) = self.state.selected_box
                    && let Some(bounding_box) = self.bounding_boxes.get_mut(index)
                {
                    bounding_box.translate(delta, self.image_size);
                }
            }
        }
    }

    fn start_keyboard_draft(&mut self, response: &Response, transform: ImageTransform) {
        let center = response
            .hover_pos()
            .map(|position| transform.screen_to_image_clamped(position, self.image_size))
            .unwrap_or_else(|| Pos2::new(self.image_size[0] * 0.5, self.image_size[1] * 0.5));
        let half_size = Vec2::splat(16.0);
        let mut draft_box =
            BoundingBox::new(center - half_size, center + half_size, *self.selected_class);
        draft_box.clip_to_image(self.image_size);
        self.state.draft_box = Some(draft_box);
        self.state.selected_box = None;
        self.state.keyboard_mode = KeyboardMode::DraftResize {
            corner: Corner::BottomRight,
        };
    }

    fn handle_pointer_input(&mut self, ui: &Ui, response: &Response, transform: ImageTransform) {
        let pointer_position = ui.input(|input| input.pointer.interact_pos());

        if response.clicked_by(PointerButton::Secondary) {
            self.delete_box_at(pointer_position, transform);
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
    }

    fn start_pointer_interaction(&mut self, position: Pos2, transform: ImageTransform) {
        if let Some((index, corner)) = self.hit_test_handle(position, transform) {
            self.state.selected_box = Some(index);
            self.state.keyboard_mode = KeyboardMode::None;
            self.state.interaction = Interaction::Resizing { index, corner };
            *self.selected_class = self.bounding_boxes[index].class;
            return;
        }

        let Some(image_position) = transform.screen_to_image(position) else {
            return;
        };

        if let Some(index) = topmost_box_at(self.bounding_boxes, image_position) {
            self.state.selected_box = Some(index);
            self.state.keyboard_mode = KeyboardMode::None;
            self.state.interaction = Interaction::Moving {
                index,
                last_position: image_position,
            };
            *self.selected_class = self.bounding_boxes[index].class;
            return;
        }

        self.state.selected_box = None;
        self.state.keyboard_mode = KeyboardMode::None;
        self.state.draft_box = Some(BoundingBox::new(
            image_position,
            image_position,
            *self.selected_class,
        ));
        self.state.interaction = Interaction::Drawing {
            start: image_position,
        };
    }

    fn update_pointer_interaction(&mut self, position: Pos2, transform: ImageTransform) {
        let image_position = transform.screen_to_image_clamped(position, self.image_size);
        match self.state.interaction {
            Interaction::None => {}
            Interaction::Drawing { start } => {
                self.state.draft_box = Some(BoundingBox::new(
                    start,
                    image_position,
                    *self.selected_class,
                ));
            }
            Interaction::Moving {
                index,
                last_position,
            } => {
                if let Some(bounding_box) = self.bounding_boxes.get_mut(index) {
                    bounding_box.translate(image_position - last_position, self.image_size);
                    self.state.interaction = Interaction::Moving {
                        index,
                        last_position: image_position,
                    };
                }
            }
            Interaction::Resizing { index, corner } => {
                if let Some(bounding_box) = self.bounding_boxes.get_mut(index) {
                    bounding_box.set_corner(corner, image_position);
                    bounding_box.clip_to_image(self.image_size);
                }
            }
        }
    }

    fn finish_pointer_interaction(&mut self) {
        if matches!(self.state.interaction, Interaction::Drawing { .. }) {
            self.commit_draft_box();
        }
        self.state.interaction = Interaction::None;
    }

    fn delete_box_at(&mut self, pointer_position: Option<Pos2>, transform: ImageTransform) {
        if let Some(position) = pointer_position
            && let Some(image_position) = transform.screen_to_image(position)
            && let Some(index) = topmost_box_at(self.bounding_boxes, image_position)
        {
            self.bounding_boxes.remove(index);
            self.state.selected_box = None;
            self.state.keyboard_mode = KeyboardMode::None;
            return;
        }

        self.delete_selected_box();
    }

    fn delete_selected_box(&mut self) {
        if let Some(index) = self.state.selected_box.take()
            && index < self.bounding_boxes.len()
        {
            self.bounding_boxes.remove(index);
        }
        self.state.keyboard_mode = KeyboardMode::None;
    }

    fn commit_draft_box(&mut self) {
        let Some(mut draft_box) = self.state.draft_box.take() else {
            return;
        };
        draft_box.clip_to_image(self.image_size);
        if draft_box.is_valid() {
            self.bounding_boxes.push(draft_box);
            self.state.selected_box = Some(self.bounding_boxes.len() - 1);
        }
        self.state.keyboard_mode = KeyboardMode::None;
    }

    fn hit_test_handle(
        &self,
        screen_position: Pos2,
        transform: ImageTransform,
    ) -> Option<(usize, Corner)> {
        self.bounding_boxes
            .iter()
            .enumerate()
            .rev()
            .find_map(|(index, bounding_box)| {
                Corner::ALL.into_iter().find_map(|corner| {
                    let corner_screen = transform.image_to_screen(bounding_box.corner(corner));
                    (corner_screen.distance(screen_position) <= HANDLE_HIT_RADIUS)
                        .then_some((index, corner))
                })
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

        for (index, bounding_box) in self.bounding_boxes.iter().enumerate() {
            self.draw_box(
                ui,
                transform,
                bounding_box,
                self.state.selected_box == Some(index),
                false,
            );
        }

        if let Some(draft_box) = &self.state.draft_box {
            self.draw_box(ui, transform, draft_box, true, true);
        }
    }

    fn draw_box(
        &self,
        ui: &Ui,
        transform: ImageTransform,
        bounding_box: &BoundingBox,
        selected: bool,
        draft: bool,
    ) {
        if !bounding_box.is_valid() && !draft {
            return;
        }

        let painter = ui.painter();
        let rect = transform.image_rect_to_screen(bounding_box.rect);
        let color = bounding_box.class.color();
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
            format!("new {}", bounding_box.class.as_str())
        } else {
            bounding_box.class.as_str().to_string()
        };
        let label_position = rect.left_top() + Vec2::new(4.0, 4.0);
        painter.text(
            label_position,
            Align2::LEFT_TOP,
            label,
            FontId::proportional(13.0),
            Color32::from_rgb(205, 214, 244),
        );

        if selected || draft {
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

fn topmost_box_at(bounding_boxes: &[BoundingBox], image_position: Pos2) -> Option<usize> {
    bounding_boxes
        .iter()
        .enumerate()
        .filter(|(_, bounding_box)| bounding_box.contains(image_position))
        .min_by(|(_, left), (_, right)| left.rect.area().total_cmp(&right.rect.area()))
        .map(|(index, _)| index)
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
