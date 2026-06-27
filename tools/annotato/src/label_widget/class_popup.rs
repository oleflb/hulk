use eframe::egui::{
    Align2, Button, Color32, FontId, Frame, Key, RichText, Stroke, Ui, Vec2, Window,
};

use crate::{classes::Class, user_toml::CONFIG};

use super::LabelWidget;

impl LabelWidget {
    pub(super) fn handle_class_popup_shortcut(&mut self, ui: &Ui) {
        let config = &CONFIG.get().unwrap().keybindings;
        if ui.input(|input| config.class_popup.is_pressed(input)) {
            if self.class_popup_open {
                self.class_popup_open = false;
            } else {
                self.open_class_popup();
            }
        }
    }

    pub(super) fn open_class_popup(&mut self) {
        self.class_popup_open = true;
        self.class_popup_index = Class::ALL
            .iter()
            .position(|class| *class == self.selected_class)
            .unwrap_or(0);
    }

    pub(super) fn class_popup_ui(&mut self, ui: &Ui) {
        if !self.class_popup_open {
            return;
        }

        let mut chosen_class = None;
        ui.input(|input| {
            if input.key_pressed(Key::ArrowDown) {
                self.class_popup_index = (self.class_popup_index + 1) % Class::ALL.len();
            }
            if input.key_pressed(Key::ArrowUp) {
                self.class_popup_index =
                    (self.class_popup_index + Class::ALL.len() - 1) % Class::ALL.len();
            }
            if input.key_pressed(Key::Escape) {
                self.class_popup_open = false;
            }
            if input.key_pressed(Key::Enter) || input.key_pressed(Key::Space) {
                chosen_class = Some(Class::ALL[self.class_popup_index]);
            }
        });

        Window::new("class-popup")
            .title_bar(false)
            .resizable(false)
            .collapsible(false)
            .anchor(Align2::CENTER_CENTER, Vec2::ZERO)
            .frame(
                Frame::popup(ui.style())
                    .fill(Color32::from_rgb(24, 24, 37))
                    .stroke(Stroke::new(1.0, Color32::from_rgb(137, 180, 250))),
            )
            .show(ui.ctx(), |ui| {
                ui.set_min_width(360.0);
                ui.vertical_centered(|ui| {
                    ui.label(
                        RichText::new("Select Class")
                            .font(FontId::proportional(24.0))
                            .strong(),
                    );
                    ui.label(RichText::new("Arrow keys, Enter/Space, Esc").size(14.0));
                });
                ui.add_space(8.0);

                for (index, class) in Class::ALL.into_iter().enumerate() {
                    let selected = index == self.class_popup_index;
                    let fill = if selected {
                        class.color().gamma_multiply(0.45)
                    } else {
                        Color32::from_rgb(31, 31, 46)
                    };
                    let stroke = if selected {
                        Stroke::new(2.0, class.color())
                    } else {
                        Stroke::new(1.0, Color32::from_rgb(69, 71, 90))
                    };
                    let label = RichText::new(class.as_str())
                        .size(20.0)
                        .strong()
                        .color(Color32::from_rgb(245, 245, 245));
                    if ui
                        .add_sized(
                            [ui.available_width(), 40.0],
                            Button::new(label).fill(fill).stroke(stroke),
                        )
                        .clicked()
                    {
                        chosen_class = Some(class);
                    }
                }
            });

        if let Some(class) = chosen_class {
            self.selected_class = class;
            self.class_popup_open = false;
        }
    }
}
