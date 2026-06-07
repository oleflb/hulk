use eframe::egui::{
    self, Align, Align2, Button, Color32, CornerRadius, FontId, Grid, Image as EguiImage, Layout,
    Rect, RichText, Sense, Stroke, Ui, Vec2, load::SizedTexture,
};
use ros_z::time::Time;

use crate::apriltag::DetectedTag;
use crate::calibration::{CalibrationResult, CalibrationSample, CalibrationState};
use crate::image::LoadedImage;

pub(crate) fn image_card(
    ui: &mut Ui,
    title: &str,
    image: &LoadedImage,
    samples: &[CalibrationSample],
    left_camera: bool,
) {
    egui::Frame::group(ui.style())
        .fill(Color32::from_rgb(24, 29, 39))
        .stroke(Stroke::new(1.0, Color32::from_rgb(54, 63, 79)))
        .corner_radius(CornerRadius::same(12))
        .show(ui, |ui| {
            ui.vertical_centered(|ui| {
                ui.label(
                    RichText::new(title)
                        .font(FontId::proportional(18.0))
                        .strong()
                        .color(Color32::from_rgb(226, 232, 240)),
                );
                ui.label(
                    RichText::new(format!("{} x {}", image.size.x as u32, image.size.y as u32))
                        .color(Color32::from_rgb(148, 163, 184)),
                );
                ui.add_space(8.0);
            });

            let available = ui.available_size_before_wrap() - Vec2::new(8.0, 8.0);
            let scale = (available.x / image.size.x)
                .min(available.y / image.size.y)
                .max(0.1);
            let size = image.size * scale;
            let texture = SizedTexture {
                id: image.texture.id(),
                size,
            };
            ui.with_layout(Layout::top_down(Align::Center), |ui| {
                let (image_rect, _) = ui.allocate_exact_size(size, Sense::hover());
                EguiImage::new(texture).paint_at(ui, image_rect);
                draw_captured_samples(ui, image_rect, image, samples, left_camera);
                draw_tags(ui, image_rect, image);
            });
        });
}

pub(crate) fn calibration_card(
    ui: &mut Ui,
    calibration: &CalibrationState,
    common_tag_count: usize,
) -> bool {
    let mut capture_clicked = false;
    egui::Frame::group(ui.style())
        .fill(Color32::from_rgb(18, 23, 33))
        .stroke(Stroke::new(1.0, Color32::from_rgb(45, 53, 67)))
        .corner_radius(CornerRadius::same(12))
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.vertical(|ui| {
                    ui.label(
                        RichText::new("Stereo calibration")
                            .font(FontId::proportional(18.0))
                            .strong()
                            .color(Color32::from_rgb(226, 232, 240)),
                    );
                    ui.label(
                        RichText::new(format!(
                            "{} samples, {} common live tag(s), tag size 5.5 cm{}",
                            calibration.samples().len(),
                            common_tag_count,
                            last_sample_text(calibration.samples()),
                        ))
                        .color(Color32::from_rgb(148, 163, 184)),
                    );
                });
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    capture_clicked = ui
                        .add_enabled(common_tag_count > 0, Button::new("Capture sample"))
                        .clicked();
                    ui.label(
                        RichText::new(calibration.status()).color(Color32::from_rgb(203, 213, 225)),
                    );
                });
            });

            if let Some(result) = calibration.result() {
                ui.add_space(10.0);
                calibration_result(ui, result);
            }
        });
    capture_clicked
}

fn last_sample_text(samples: &[CalibrationSample]) -> String {
    samples.last().map_or_else(String::new, |sample| {
        format!(
            ", last frame {}, tag {}",
            sample.frame_identifier, sample.tag_id
        )
    })
}

pub(crate) fn info_card(ui: &mut Ui, label: &str, value: &str) {
    egui::Frame::group(ui.style())
        .fill(Color32::from_rgb(18, 23, 33))
        .stroke(Stroke::new(1.0, Color32::from_rgb(45, 53, 67)))
        .corner_radius(CornerRadius::same(10))
        .show(ui, |ui| {
            ui.set_min_width(150.0);
            ui.label(RichText::new(label).color(Color32::from_rgb(129, 140, 157)));
            ui.label(
                RichText::new(value)
                    .font(FontId::monospace(14.0))
                    .color(Color32::from_rgb(226, 232, 240)),
            );
        });
}

pub(crate) fn status_pill(ui: &mut Ui, status: &str) {
    let color = if status.starts_with("matched") {
        Color32::from_rgb(34, 197, 94)
    } else if status.starts_with("failed") {
        Color32::from_rgb(248, 113, 113)
    } else {
        Color32::from_rgb(251, 191, 36)
    };
    egui::Frame::group(ui.style())
        .fill(Color32::from_rgb(16, 22, 32))
        .stroke(Stroke::new(1.0, color))
        .corner_radius(CornerRadius::same(16))
        .show(ui, |ui| {
            ui.label(RichText::new(status).color(color));
        });
}

pub(crate) fn empty_state(ui: &mut Ui) {
    ui.vertical_centered(|ui| {
        ui.add_space(90.0);
        ui.label(
            RichText::new("No matched stereo pair yet")
                .font(FontId::proportional(24.0))
                .strong()
                .color(Color32::from_rgb(226, 232, 240)),
        );
        ui.add_space(6.0);
        ui.label(
            RichText::new(
                "The viewer only displays frames when left and right timestamps match exactly.",
            )
            .color(Color32::from_rgb(148, 163, 184)),
        );
    });
}

pub(crate) fn format_stamp(stamp: Time) -> String {
    let nanos = stamp.as_nanos();
    format!("{}.{:09}", nanos / 1_000_000_000, nanos % 1_000_000_000)
}

fn draw_tags(ui: &Ui, rect: Rect, image: &LoadedImage) {
    let scale = Vec2::new(rect.width() / image.size.x, rect.height() / image.size.y);
    let stroke = Stroke::new(2.5, Color32::from_rgb(45, 212, 191));
    for tag in &image.tags {
        let corners = tag
            .corners
            .map(|p| rect.min + Vec2::new(p.x * scale.x, p.y * scale.y));
        draw_tag(ui, corners, tag, stroke);
    }
}

fn draw_captured_samples(
    ui: &Ui,
    rect: Rect,
    image: &LoadedImage,
    samples: &[CalibrationSample],
    left_camera: bool,
) {
    let scale = Vec2::new(rect.width() / image.size.x, rect.height() / image.size.y);
    let stroke = Stroke::new(1.5, Color32::from_rgba_premultiplied(147, 197, 253, 130));
    for sample in samples
        .iter()
        .filter(|sample| sample.image_size == image.size)
    {
        let source = if left_camera {
            sample.left
        } else {
            sample.right
        };
        let corners = source.map(|p| rect.min + Vec2::new(p.x * scale.x, p.y * scale.y));
        draw_boundary(ui, corners, stroke);
    }
}

fn draw_tag(ui: &Ui, corners: [egui::Pos2; 4], tag: &DetectedTag, stroke: Stroke) {
    draw_boundary(ui, corners, stroke);
    ui.painter().text(
        corners[0],
        Align2::LEFT_TOP,
        tag.id.to_string(),
        FontId::monospace(14.0),
        Color32::WHITE,
    );
}

fn draw_boundary(ui: &Ui, corners: [egui::Pos2; 4], stroke: Stroke) {
    for i in 0..4 {
        ui.painter()
            .line_segment([corners[i], corners[(i + 1) % 4]], stroke);
    }
}

fn calibration_result(ui: &mut Ui, result: &CalibrationResult) {
    Grid::new("calibration-result")
        .num_columns(2)
        .spacing([18.0, 6.0])
        .show(ui, |ui| {
            result_row(ui, "RMS", format!("{:.6}", result.rms));
            result_row(ui, "Baseline", format!("{:.4} m", result.baseline));
            result_row(ui, "Left K", format_matrix(&result.left_camera_matrix, 3));
            result_row(ui, "Left dist", format_vector(&result.left_dist_coeffs));
            result_row(ui, "Right K", format_matrix(&result.right_camera_matrix, 3));
            result_row(ui, "Right dist", format_vector(&result.right_dist_coeffs));
            result_row(ui, "R", format_matrix(&result.rotation, 3));
            result_row(ui, "T", format_vector(&result.translation));
        });
}

fn result_row(ui: &mut Ui, label: &str, value: String) {
    ui.label(RichText::new(label).color(Color32::from_rgb(148, 163, 184)));
    ui.label(
        RichText::new(value)
            .font(FontId::monospace(13.0))
            .color(Color32::from_rgb(226, 232, 240)),
    );
    ui.end_row();
}

fn format_matrix(values: &[f64], width: usize) -> String {
    values
        .chunks(width)
        .map(format_vector)
        .collect::<Vec<_>>()
        .join("  ")
}

fn format_vector(values: &[f64]) -> String {
    format!(
        "[{}]",
        values
            .iter()
            .map(|value| format!("{value:.5}"))
            .collect::<Vec<_>>()
            .join(", ")
    )
}
