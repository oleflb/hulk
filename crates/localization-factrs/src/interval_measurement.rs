use crate::measurements::{ImuMeasurement, VisualMeasurement};

pub struct IntervalMeasurements {
    pub imu: Vec<ImuMeasurement>,
    pub visual: Vec<Vec<VisualMeasurement>>,
}

impl IntervalMeasurements {
    pub fn new() -> Self {
        Self {
            imu: Vec::new(),
            visual: Vec::new(),
        }
    }

    pub fn push_imu(&mut self, imu: ImuMeasurement) {
        insert_sorted(&mut self.imu, imu, |imu| imu.time);
    }

    pub fn push_visual(&mut self, visual: Vec<VisualMeasurement>) {
        insert_sorted(&mut self.visual, visual, |visual| {
            visual
                .first()
                .expect("visual frames must contain at least one measurement")
                .time
        });
    }
}

fn insert_sorted<T, K: Ord>(vec: &mut Vec<T>, item: T, key: impl Fn(&T) -> K) {
    let index = vec
        .binary_search_by_key(&key(&item), key)
        .unwrap_or_else(|index| index);
    vec.insert(index, item);
}
