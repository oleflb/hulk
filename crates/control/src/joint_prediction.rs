use std::collections::VecDeque;

use color_eyre::Result;
use context_attribute::context;
use framework::MainOutput;
use inference::LinearRegressor;
use nalgebra::{vector, ArrayStorage, SVector, Vector6, U26};
use serde::{Deserialize, Serialize};
use types::{cycle_time::CycleTime, sensor_data::SensorData};

const K_HISTORY_LENGTH: usize = 26;

#[derive(Deserialize, Serialize)]
pub struct JointPrediction {
    historic_sensor_data: VecDeque<SensorData>,
    gyroscope_y_predictor: LinearRegressor<U26, ArrayStorage<f32, K_HISTORY_LENGTH, 1>>,
}

#[context]
pub struct CreationContext {}

#[context]
pub struct CycleContext {
    cycle_time: Input<CycleTime, "cycle_time">,
    sensor_data: Input<SensorData, "sensor_data">,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub gyroscope_y: MainOutput<f32>,
}
impl JointPrediction {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self {
            historic_sensor_data: VecDeque::with_capacity(K_HISTORY_LENGTH),
            gyroscope_y_predictor: LinearRegressor::new(
                vector![
                    -0.21725520804207138,
                    -0.0842471662079518,
                    -0.10257492375291168,
                    -0.08645591499198309,
                    -0.09340833730821244,
                    0.027943919823220243,
                    0.18851218297722647,
                    0.10182204784546454,
                    0.2031363249635547,
                    0.07488167765165113,
                    -0.1063791930984413,
                    0.09427154144363768,
                    -0.17896031423948572,
                    0.058470571402770864,
                    -0.17627060750201196,
                    0.0781867472711869,
                    -0.19567450816431584,
                    0.11852236703664176,
                    -0.11108034217302117,
                    -0.04072956725089671,
                    -0.26954875239719917,
                    0.2155764469895482,
                    -0.019344949633403583,
                    -0.08853436329766634,
                    -0.02850638950389145,
                    0.4876846696469047,
                ],
                2.5042150518191113e-05,
            ),
        })
    }

    pub fn cycle(&mut self, context: CycleContext) -> Result<MainOutputs> {
        self.historic_sensor_data
            .push_back(context.sensor_data.clone());
        if self.historic_sensor_data.len() > K_HISTORY_LENGTH {
            self.historic_sensor_data.pop_front();
        }

        let sensor_datas = gather_sensor_data(&self.historic_sensor_data);
        let feature_vector = to_feature_vector(&sensor_datas);
        let prediction = self.gyroscope_y_predictor.predict(&feature_vector);

        Ok(MainOutputs {
            gyroscope_y: prediction.into(),
        })
    }
}

fn gather_sensor_data(sensor_datas: &VecDeque<SensorData>) -> Vec<&SensorData> {
    // get the last 6 sensor datas, if there are less than K_HISTORY_LENGTH, pad with the first one
    let mut sensor_datas = sensor_datas.iter().collect::<Vec<_>>();
    while sensor_datas.len() < K_HISTORY_LENGTH {
        sensor_datas.push(sensor_datas[0]);
    }
    sensor_datas
}

fn to_feature_vector(sensor_datas: &[&SensorData]) -> SVector<f32, K_HISTORY_LENGTH> {
    let mut feature_vector = SVector::zeros();
    for (i, sensor_data) in sensor_datas.iter().enumerate() {
        feature_vector[i] = sensor_data.inertial_measurement_unit.angular_velocity.y();
    }
    feature_vector
}
