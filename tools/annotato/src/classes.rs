use eframe::{egui::Key, epaint::Color32};
use serde::{Deserialize, Serialize};

use crate::{user_toml::CONFIG, widgets::class_selector::EnumIter};

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize, Hash)]
pub enum Class {
    Ball,
    Robot,
    GoalPost,
    PenaltySpot,
    LSpot,
    TSpot,
    XSpot,
    Person,
}

impl EnumIter for Class {
    fn list() -> Vec<Self> {
        use Class::*;
        vec![
            Ball,
            Robot,
            GoalPost,
            PenaltySpot,
            LSpot,
            TSpot,
            XSpot,
            Person,
        ]
    }
}

impl Class {
    pub fn supports_boxes(self) -> bool {
        !self.requires_point()
    }

    pub fn supports_points(self) -> bool {
        self.requires_point() || self == Class::GoalPost
    }

    pub fn requires_point(self) -> bool {
        matches!(self, Class::LSpot | Class::TSpot | Class::XSpot)
    }

    pub fn from_key(key: Key) -> Option<Class> {
        let keybindings = &CONFIG.get().unwrap().keybindings;
        match key {
            x if x == keybindings.select_ball => Some(Class::Ball),
            x if x == keybindings.select_robot => Some(Class::Robot),
            x if x == keybindings.select_goalpost => Some(Class::GoalPost),
            x if x == keybindings.select_penaltyspot => Some(Class::PenaltySpot),
            x if x == keybindings.select_lspot => Some(Class::LSpot),
            x if x == keybindings.select_tspot => Some(Class::TSpot),
            x if x == keybindings.select_xspot => Some(Class::XSpot),
            x if x == keybindings.select_person => Some(Class::Person),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Class::Ball => "Ball",
            Class::Robot => "Robot",
            Class::GoalPost => "Goal post",
            Class::PenaltySpot => "Penalty spot",
            Class::LSpot => "L spot",
            Class::TSpot => "T spot",
            Class::XSpot => "X spot",
            Class::Person => "Person",
        }
    }

    pub fn next(self) -> Self {
        let classes = Self::list();
        let index = classes.iter().position(|class| *class == self).unwrap_or(0);
        classes[(index + 1) % classes.len()]
    }

    pub fn previous(self) -> Self {
        let classes = Self::list();
        let index = classes.iter().position(|class| *class == self).unwrap_or(0);
        classes[(index + classes.len() - 1) % classes.len()]
    }

    pub fn color(&self) -> Color32 {
        match self {
            Class::Robot => Color32::from_rgb(137, 180, 250),
            Class::Ball => Color32::from_rgb(250, 179, 135),
            Class::GoalPost => Color32::from_rgb(243, 139, 168),
            Class::PenaltySpot => Color32::from_rgb(249, 226, 175),
            Class::LSpot => Color32::from_rgb(203, 166, 247),
            Class::TSpot => Color32::from_rgb(166, 227, 161),
            Class::XSpot => Color32::from_rgb(137, 220, 235),
            Class::Person => Color32::from_rgb(245, 194, 231),
        }
    }
}
