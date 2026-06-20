use color_eyre::Result;
use eframe::{
    App, CreationContext, Frame,
    egui::{self, Align, CentralPanel, Color32, FontId, Layout, RichText, TopBottomPanel},
};
use opencv::objdetect;
use ros_z::time::Time;
use types::stereo_image_pair::StereoImagePair;

use crate::calibration::{CalibrationState, common_tags};
use crate::image::{LoadedImage, load_image};
use crate::ros::RosState;
use crate::ui::{calibration_card, empty_state, format_stamp, image_card, info_card, status_pill};

pub(crate) struct CalibrationApp {
    ros: RosState,
    detector: objdetect::ArucoDetector,
    current_match: Option<StereoMatch>,
    calibration: CalibrationState,
    status: String,
}

struct StereoMatch {
    stamp: Time,
    frame_identifier: u32,
    left: LoadedImage,
    right: LoadedImage,
}

impl CalibrationApp {
    pub(crate) fn new(
        creation_context: &CreationContext,
        ros: RosState,
        detector: objdetect::ArucoDetector,
    ) -> Self {
        creation_context.egui_ctx.set_visuals(egui::Visuals::dark());
        Self {
            ros,
            detector,
            current_match: None,
            calibration: CalibrationState::default(),
            status: "waiting for stereo image pairs".to_string(),
        }
    }

    fn update_match(&mut self, context: &egui::Context) {
        if let Some(error) = self.ros.stereo_subscription_error() {
            self.status = error;
            return;
        }

        let Some(pair) = self.ros.latest_stereo_pair() else {
            self.status = self.ros.stereo_wait_status();
            return;
        };

        if self
            .current_match
            .as_ref()
            .is_some_and(|m| m.frame_identifier == pair.value.inner.frame_identifier)
        {
            return;
        }

        match self.load_match(context, pair.value.time, &pair.value.inner) {
            Ok(match_) => {
                self.status = format!(
                    "matched stereo pair, tags left: {}, right: {}",
                    match_.left.tags.len(),
                    match_.right.tags.len()
                );
                self.current_match = Some(match_);
            }
            Err(error) => self.status = format!("failed to display stereo pair: {error}"),
        }
    }

    fn load_match(
        &self,
        context: &egui::Context,
        stamp: Time,
        pair: &StereoImagePair,
    ) -> Result<StereoMatch> {
        Ok(StereoMatch {
            stamp,
            frame_identifier: pair.frame_identifier,
            left: load_image(context, &self.detector, "intrinsic-left", &pair.left)?,
            right: load_image(context, &self.detector, "intrinsic-right", &pair.right)?,
        })
    }
}

impl App for CalibrationApp {
    fn update(&mut self, context: &egui::Context, _frame: &mut Frame) {
        context.request_repaint();
        self.update_match(context);
        let mut capture_request = None;

        TopBottomPanel::top("header").show(context, |ui| {
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                ui.vertical(|ui| {
                    ui.label(
                        RichText::new("Intrinsic Calibration")
                            .font(FontId::proportional(26.0))
                            .strong()
                            .color(Color32::from_rgb(236, 239, 244)),
                    );
                    ui.label(
                        RichText::new("Live stereo image alignment")
                            .color(Color32::from_rgb(150, 159, 175)),
                    );
                });
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    status_pill(ui, &self.status);
                });
            });
            ui.add_space(8.0);
        });

        CentralPanel::default().show(context, |ui| {
            ui.add_space(10.0);
            ui.horizontal(|ui| {
                info_card(ui, "Robot", &self.ros.namespace);
                info_card(ui, "Stereo", &self.ros.stereo_topic);
                if let Some(matched) = &self.current_match {
                    info_card(ui, "Frame", &matched.frame_identifier.to_string());
                    info_card(ui, "Stamp", &format_stamp(matched.stamp));
                }
            });
            ui.add_space(12.0);

            let Some(match_) = &self.current_match else {
                empty_state(ui);
                return;
            };

            let common_tag_count = common_tags(&match_.left.tags, &match_.right.tags).len();
            let capture_clicked = calibration_card(ui, &self.calibration, common_tag_count);
            if capture_clicked {
                capture_request = Some((
                    match_.frame_identifier,
                    match_.left.size,
                    match_.left.tags.clone(),
                    match_.right.tags.clone(),
                ));
            }

            ui.add_space(12.0);

            ui.columns(2, |columns| {
                image_card(
                    &mut columns[0],
                    "Left camera",
                    &match_.left,
                    self.calibration.samples(),
                    true,
                );
                image_card(
                    &mut columns[1],
                    "Right camera",
                    &match_.right,
                    self.calibration.samples(),
                    false,
                );
            });
        });

        if let Some((frame_identifier, image_size, left_tags, right_tags)) = capture_request {
            let added =
                self.calibration
                    .capture(frame_identifier, image_size, &left_tags, &right_tags);
            self.status = match added {
                0 => "no new calibration samples captured".to_string(),
                1 => "captured 1 calibration sample".to_string(),
                _ => format!("captured {added} calibration samples"),
            };
        }
    }
}
