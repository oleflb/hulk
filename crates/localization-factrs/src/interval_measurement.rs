use crate::{
    factors::{
        foot_above_ground::FootHeightMeasurement, visual_odometry::VisualOdometryMeasurement,
    },
    measurements::{ImuMeasurement, SensorMeasurement, VisualReprojectionMeasurement},
};

pub struct IntervalMeasurements {
    pub imu: Vec<ImuMeasurement>,
    pub visual: Vec<Vec<VisualReprojectionMeasurement>>,
    pub visual_odometry: Vec<VisualOdometryMeasurement>,
    pub foot_heights: Vec<FootHeightMeasurement>,
}

impl IntervalMeasurements {
    pub fn new() -> Self {
        Self {
            imu: Vec::new(),
            visual: Vec::new(),
            visual_odometry: Vec::new(),
            foot_heights: Vec::new(),
        }
    }

    pub fn push_imu(&mut self, imu: ImuMeasurement) {
        insert_sorted(&mut self.imu, imu, |imu| imu.time);
    }

    pub fn push(&mut self, measurement: SensorMeasurement) {
        match measurement {
            SensorMeasurement::Imu(imu) => self.push_imu(imu),
            SensorMeasurement::Visual(visual) => self.push_visual(visual),
            SensorMeasurement::VisualOdometry(visual_odometry) => {
                self.push_visual_odometry(visual_odometry)
            }
            SensorMeasurement::FootHeights(foot_heights) => self.push_foot_heights(foot_heights),
        }
    }

    pub fn push_visual(&mut self, visual: Vec<VisualReprojectionMeasurement>) {
        insert_sorted(&mut self.visual, visual, |visual| {
            visual
                .first()
                .expect("visual frames must contain at least one measurement")
                .time
        });
    }

    pub fn push_visual_odometry(&mut self, visual_odometry: VisualOdometryMeasurement) {
        insert_sorted(&mut self.visual_odometry, visual_odometry, |measurement| {
            measurement.current_time
        });
    }

    pub fn push_foot_heights(&mut self, foot_heights: FootHeightMeasurement) {
        insert_sorted(&mut self.foot_heights, foot_heights, |measurement| {
            measurement.time
        });
    }
}

fn insert_sorted<T, K: Ord>(vec: &mut Vec<T>, item: T, key: impl Fn(&T) -> K) {
    let index = vec
        .binary_search_by_key(&key(&item), key)
        .unwrap_or_else(|index| index);
    vec.insert(index, item);
}
