use eframe::egui::{ComboBox, Id, Response, Ui, Widget};

use crate::classes::Class;
pub trait EnumIter {
    fn list() -> Vec<Self>
    where
        Self: Sized;
}

pub struct ClassSelector<'a> {
    id: Id,
    currently_selected: &'a mut Class,
}

impl<'a> ClassSelector<'a> {
    pub fn new(id_source: impl Into<Id>, currently_selected: &'a mut Class) -> Self {
        Self {
            id: id_source.into(),
            currently_selected,
        }
    }
}

impl Widget for ClassSelector<'_> {
    fn ui(self, ui: &mut Ui) -> Response {
        ComboBox::from_id_salt(self.id)
            .selected_text(self.currently_selected.as_str())
            .show_ui(ui, |ui| {
                Class::list().into_iter().for_each(|class| {
                    ui.selectable_value(self.currently_selected, class, class.as_str());
                });
            })
            .response
    }
}
