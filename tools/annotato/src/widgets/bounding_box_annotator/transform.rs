use eframe::egui::{Pos2, Rect, Vec2};

#[derive(Debug, Clone, Copy)]
pub(super) struct ImageTransform {
    pub(super) image_rect: Rect,
    scale: f32,
}

impl ImageTransform {
    pub(super) fn new(canvas_rect: Rect, image_size: [f32; 2], zoom: f32, pan: Vec2) -> Self {
        let scale = Self::scale(canvas_rect, image_size, zoom);
        let displayed_size = Vec2::new(image_size[0] * scale, image_size[1] * scale);
        let image_rect = Rect::from_center_size(canvas_rect.center() + pan, displayed_size);

        Self { image_rect, scale }
    }

    fn scale(canvas_rect: Rect, image_size: [f32; 2], zoom: f32) -> f32 {
        let padding = Vec2::splat(16.0);
        let available = (canvas_rect.size() - 2.0 * padding).max(Vec2::splat(1.0));
        let base_scale = (available.x / image_size[0])
            .min(available.y / image_size[1])
            .max(0.001);
        base_scale * zoom.max(1.0)
    }

    pub(super) fn pan_for_anchor(
        canvas_rect: Rect,
        image_size: [f32; 2],
        zoom: f32,
        screen_anchor: Pos2,
        image_anchor: Pos2,
    ) -> Vec2 {
        let scale = Self::scale(canvas_rect, image_size, zoom);
        let displayed_size = Vec2::new(image_size[0] * scale, image_size[1] * scale);
        let top_left = screen_anchor - Vec2::new(image_anchor.x * scale, image_anchor.y * scale);
        let center = top_left + displayed_size * 0.5;
        center - canvas_rect.center()
    }

    pub(super) fn screen_to_image(self, screen_position: Pos2) -> Option<Pos2> {
        self.image_rect.contains(screen_position).then(|| {
            Pos2::new(
                (screen_position.x - self.image_rect.left()) / self.scale,
                (screen_position.y - self.image_rect.top()) / self.scale,
            )
        })
    }

    pub(super) fn screen_to_image_unclamped(self, screen_position: Pos2) -> Pos2 {
        Pos2::new(
            (screen_position.x - self.image_rect.left()) / self.scale,
            (screen_position.y - self.image_rect.top()) / self.scale,
        )
    }

    pub(super) fn screen_to_image_clamped(
        self,
        screen_position: Pos2,
        image_size: [f32; 2],
    ) -> Pos2 {
        Pos2::new(
            ((screen_position.x - self.image_rect.left()) / self.scale).clamp(0.0, image_size[0]),
            ((screen_position.y - self.image_rect.top()) / self.scale).clamp(0.0, image_size[1]),
        )
    }

    pub(super) fn image_to_screen(self, image_position: Pos2) -> Pos2 {
        Pos2::new(
            self.image_rect.left() + image_position.x * self.scale,
            self.image_rect.top() + image_position.y * self.scale,
        )
    }

    pub(super) fn image_rect_to_screen(self, image_rect: Rect) -> Rect {
        Rect::from_min_max(
            self.image_to_screen(image_rect.min),
            self.image_to_screen(image_rect.max),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn anchored_zoom_keeps_image_point_under_cursor() {
        let canvas = Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 800.0));
        let image_size = [200.0, 100.0];
        let pointer = Pos2::new(350.0, 420.0);
        let base = ImageTransform::new(canvas, image_size, 1.0, Vec2::ZERO);
        let image_anchor = base.screen_to_image_unclamped(pointer);

        let pan = ImageTransform::pan_for_anchor(canvas, image_size, 4.0, pointer, image_anchor);
        let zoomed = ImageTransform::new(canvas, image_size, 4.0, pan);

        assert!(zoomed.image_to_screen(image_anchor).distance(pointer) < 0.001);
    }
}

pub(super) fn clamp_point(point: Pos2, image_size: [f32; 2]) -> Pos2 {
    Pos2::new(
        point.x.clamp(0.0, image_size[0]),
        point.y.clamp(0.0, image_size[1]),
    )
}
