use clap::ValueEnum;

#[derive(Clone, Copy, Debug, Eq, PartialEq, ValueEnum)]
pub(crate) enum CameraSelection {
    Left,
    Right,
    Both,
}

impl CameraSelection {
    pub(crate) fn sides(self) -> &'static [CameraSide] {
        match self {
            Self::Left => &[CameraSide::Left],
            Self::Right => &[CameraSide::Right],
            Self::Both => &[CameraSide::Left, CameraSide::Right],
        }
    }
}

impl std::fmt::Display for CameraSelection {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Left => formatter.write_str("left"),
            Self::Right => formatter.write_str("right"),
            Self::Both => formatter.write_str("both"),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum CameraSide {
    Left,
    Right,
}

impl CameraSide {
    pub(crate) fn label(self) -> &'static str {
        match self {
            Self::Left => "Left",
            Self::Right => "Right",
        }
    }

    pub(crate) fn topic(self) -> &'static str {
        match self {
            Self::Left => "inputs/left_encoded_frame",
            Self::Right => "inputs/right_encoded_frame",
        }
    }

    pub(crate) fn texture_name(self) -> &'static str {
        match self {
            Self::Left => "camera-viewer-left",
            Self::Right => "camera-viewer-right",
        }
    }
}
