use eframe::epaint::Color32;
use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize, Hash)]
pub enum Class {
    Ball,
    Robot,
    GoalPost,
    PenaltySpot,
    LSpot,
    TSpot,
    XSpot,
    #[serde(other)]
    Unknown,
}

impl Class {
    pub const ALL: [Self; 7] = [
        Self::Ball,
        Self::GoalPost,
        Self::LSpot,
        Self::PenaltySpot,
        Self::Robot,
        Self::TSpot,
        Self::XSpot,
    ];

    pub fn supports_boxes(self) -> bool {
        !self.requires_point()
    }

    pub fn supports_points(self) -> bool {
        self.requires_point()
    }

    pub fn requires_point(self) -> bool {
        matches!(
            self,
            Class::LSpot | Class::TSpot | Class::XSpot | Class::PenaltySpot | Class::GoalPost
        )
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
            Class::Unknown => "Unknown",
        }
    }

    pub fn next(self) -> Self {
        let index = Self::ALL
            .iter()
            .position(|class| *class == self)
            .unwrap_or(0);
        Self::ALL[(index + 1) % Self::ALL.len()]
    }

    pub fn previous(self) -> Self {
        let index = Self::ALL
            .iter()
            .position(|class| *class == self)
            .unwrap_or(0);
        Self::ALL[(index + Self::ALL.len() - 1) % Self::ALL.len()]
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
            Class::Unknown => Color32::DEBUG_COLOR,
        }
    }
}
