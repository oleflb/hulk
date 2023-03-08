use std::time::Duration;

use serde::{Deserialize, Deserializer, Serialize, Serializer, de::Error};

use crate::{Joints, HeadJoints, ArmJoints, LegJoints};

#[derive(Serialize, Deserialize)]
pub struct MotionFile2 {
    motion_name: String,
    initial: Joints,
    #[serde(deserialize_with = "deserialize_frame")]
    frames: Vec<MotionFileFrame>,
}

fn deserialize_frame<'de, D>(deserializer: D) -> Result<Vec<MotionFileFrame>, D::Error>
where
    D: Deserializer<'de>,
{
    let partial_frames: Vec<PartialMotionFileFrame> = Deserialize::deserialize(deserializer)?;
    let current_frame = partial_frames.first().ok_or(Error::custom("motion file contains no frames"))?;
    let current_frame = current_frame.try_into().map_err(|err| Error::custom(format!("first frame must contain full joints, got: {err}")))?;

    for frame in partial_frames.iter_mut().skip(1) {
        frame.fill_missing(current_frame);
        current_frame = frame;
    }
    Ok(Vec::new())
}


impl MotionFile2 {}

#[derive(Serialize, Deserialize)]
pub struct MotionFileFrame {
    #[serde(
        serialize_with = "serialize_float_seconds",
        deserialize_with = "deserialize_float_seconds"
    )]
    duration: Duration,
    position_update: Joints,
    conditions: ConditionDescriptor,
}

#[derive(Serialize, Deserialize)]
pub struct PartialMotionFileFrame {
    #[serde(
        serialize_with = "serialize_float_seconds",
        deserialize_with = "deserialize_float_seconds"
    )]
    duration: Duration,
    position_update: PartialJoints,
    conditions: ConditionDescriptor,
}

impl PartialMotionFileFrame{
    fn fill_missing(&mut self, other: &MotionFileFrame){
        self.position_update.fill_missing(other.position_update)
    }
}

#[derive(Serialize, Deserialize)]
pub enum ConditionDescriptor {
    #[serde(rename = "stabilize")]
    Stabilize { tolerance: f32 },
    #[serde(rename = "wait")]
    Wait {
        #[serde(
            serialize_with = "serialize_float_seconds",
            deserialize_with = "deserialize_float_seconds"
        )]
        duration: Duration,
    },
}

fn serialize_float_seconds<S>(duration: &Duration, serializer: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    serializer.serialize_f32(duration.as_secs_f32())
}

fn deserialize_float_seconds<'de, D>(deserializer: D) -> Result<Duration, D::Error>
where
    D: Deserializer<'de>,
{
    Ok(Duration::from_secs_f32(f32::deserialize(deserializer)?))
}

#[derive(Serialize, Deserialize)]
pub struct PartialJoints {
    #[serde(default = "PartialHeadJoints::new_nan")]
    pub head: PartialHeadJoints,
    #[serde(default = "PartialArmJoints::new_nan")]
    pub left_arm: PartialArmJoints,
    #[serde(default = "PartialArmJoints::new_nan")]
    pub right_arm: PartialArmJoints,
    #[serde(default = "PartialLegJoints::new_nan")]
    pub left_leg: PartialLegJoints,
    #[serde(default = "PartialLegJoints::new_nan")]
    pub right_leg: PartialLegJoints,
}

impl TryInto<Joints> for PartialJoints{
    type Error = String;

    fn try_into(self) -> Result<Joints, Self::Error> {
        Ok(Joints {
            head: self.head.try_into()?,
            left_arm: self.left_arm.try_into()?,
            right_arm: self.right_arm.try_into()?,
            left_leg: self.left_leg.try_into()?,
            right_leg: self.right_leg.try_into()?,
        })
    }
}

impl PartialJoints{
    pub fn fill_missing() {}
}

fn f32_NAN() -> f32 {
    return f32::NAN;
}
#[derive(Serialize, Deserialize)]
pub struct PartialHeadJoints {
    #[serde(default = "f32_NAN")]
    pub yaw: f32,
    #[serde(default = "f32_NAN")]
    pub pitch: f32,
}

impl TryInto<HeadJoints> for PartialHeadJoints {
    type Error = String;

    fn try_into(self) -> Result<HeadJoints, Self::Error> {
        if self.yaw.is_nan() || self.pitch.is_nan() {
            Err("HeadJoints contains NAN values")
        }
        Ok(HeadJoints{ yaw: self.yaw, pitch: self.pitch })
    }
}

impl PartialHeadJoints {
    pub const NAN: Self = Self {
        yaw: f32::NAN,
        pitch: f32::NAN,
    };

    pub const fn new_nan() -> Self {
        Self::NAN
    }
}

#[derive(Serialize, Deserialize)]
pub struct PartialArmJoints {
    #[serde(default = "f32_NAN")]
    pub shoulder_pitch: f32,
    #[serde(default = "f32_NAN")]
    pub shoulder_roll: f32,
    #[serde(default = "f32_NAN")]
    pub elbow_yaw: f32,
    #[serde(default = "f32_NAN")]
    pub elbow_roll: f32,
    #[serde(default = "f32_NAN")]
    pub wrist_yaw: f32,
    #[serde(default = "f32_NAN")]
    pub hand: f32,
}

impl TryInto<ArmJoints> for PartialArmJoints {
    type Error = String;

    fn try_into(self) -> Result<ArmJoints, Self::Error> {
        if self.shoulder_pitch.is_nan() || self.shoulder_roll.is_nan() || self.elbow_yaw.is_nan() || self.elbow_roll.is_nan() || self.wrist_yaw.is_nan() || self.hand.is_nan() {
            Err("ArmJoints contains NAN values")
        }
        Ok(ArmJoints{ shoulder_pitch: self.shoulder_pitch, shoulder_roll: self.shoulder_roll, elbow_yaw: self.elbow_yaw, elbow_roll: self.elbow_roll, wrist_yaw: self.wrist_yaw, hand: self.hand})
    }
}

impl PartialArmJoints {
    pub const NAN: Self = Self {
        shoulder_pitch: f32::NAN,
        shoulder_roll: f32::NAN,
        elbow_yaw: f32::NAN,
        elbow_roll: f32::NAN,
        wrist_yaw: f32::NAN,
        hand: f32::NAN,
    };

    pub const fn new_nan() -> Self {
        Self::NAN
    }
}

#[derive(Serialize, Deserialize)]
pub struct PartialLegJoints {
    pub hip_yaw_pitch: f32,
    pub hip_roll: f32,
    pub hip_pitch: f32,
    pub knee_pitch: f32,
    pub ankle_pitch: f32,
    pub ankle_roll: f32,
}

impl TryInto<LegJoints> for PartialLegJoints {
    type Error = String;

    fn try_into(self) -> Result<LegJoints, Self::Error> {
        if self.hip_yaw_pitch.is_nan() || self.hip_roll.is_nan() || self.hip_pitch.is_nan() || self.knee_pitch.is_nan() || self.ankle_pitch.is_nan() || self.ankle_roll.is_nan() {
            Err("LegJoints contains NAN values")
        }
        Ok(LegJoints { hip_yaw_pitch: self.hip_yaw_pitch, hip_roll: self.hip_roll, hip_pitch: self.hip_pitch, knee_pitch: self.knee_pitch, ankle_pitch: self.ankle_pitch, ankle_roll: self.ankle_roll })
    }
}

impl PartialLegJoints {
    pub const NAN: Self = Self {
        hip_yaw_pitch: f32::NAN,
        hip_roll: f32::NAN,
        hip_pitch: f32::NAN,
        knee_pitch: f32::NAN,
        ankle_pitch: f32::NAN,
        ankle_roll: f32::NAN,
    };
    
    pub const fn new_nan() -> Self {
        Self::NAN
    }
}
