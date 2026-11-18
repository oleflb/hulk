use std::time::Duration;

use coordinate_systems::{Field, Ground, Robot};
use eframe::egui::{ComboBox, Popup, PopupCloseBehavior, Ui};
use linear_algebra::{Isometry2, Isometry3, point, vector};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use types::{
    field_dimensions::FieldDimensions,
    localization::{LocalizationEstimate, LocalizationStatus, ground_to_field_from_field_to_robot},
    time_wrapper::TimeWrapper,
};

use self::observation::MapData;
use super::map_observation::MapObservation;
use crate::panel::{Panel, PanelCreationContext, PanelUiContext};
use twix_visualization::{
    twix_painter::{Orientation, TwixPainter},
    zoom_and_pan::ZoomAndPanTransform,
};

use self::layer::{EnabledLayer, Layer};

mod layer;
mod layers;
mod observation;

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
enum PlotType {
    Field,
    Ground,
}

trait GenericLayer {
    fn generic_paint(
        &mut self,
        painter: &TwixPainter<Field>,
        ground_to_field: Isometry2<Ground, Field>,
        field_dimensions: &FieldDimensions,
    );
}

impl<T: Layer<Field>> GenericLayer for EnabledLayer<T, Field> {
    fn generic_paint(
        &mut self,
        painter: &TwixPainter<Field>,
        _ground_to_field: Isometry2<Ground, Field>,
        field_dimensions: &FieldDimensions,
    ) {
        self.paint(painter, field_dimensions)
    }
}

impl<T: Layer<Ground>> GenericLayer for EnabledLayer<T, Ground> {
    fn generic_paint(
        &mut self,
        painter: &TwixPainter<Field>,
        ground_to_field: Isometry2<Ground, Field>,
        field_dimensions: &FieldDimensions,
    ) {
        self.paint(
            &painter.transform_painter(ground_to_field.inverse()),
            field_dimensions,
        )
    }
}

pub struct MapPanel {
    current_plot_type: PlotType,

    observations: Result<MapObservation, String>,
    data: MapData,
    zoom_and_pan: ZoomAndPanTransform,

    field: EnabledLayer<layers::Field, Field>,
    ball_search_heatmap: EnabledLayer<layers::BallSearchHeatmap, Field>,
    path_obstacles: EnabledLayer<layers::PathObstacles, Ground>,
    obstacles: EnabledLayer<layers::Obstacles, Ground>,
    path: EnabledLayer<layers::Path, Ground>,
    robot_pose: EnabledLayer<layers::RobotPose, Ground>,
    odometry: EnabledLayer<layers::Odometry, Field>,
    ball_percepts: EnabledLayer<layers::BallPercepts, Ground>,
    ball_position: EnabledLayer<layers::BallPosition, Field>,
    ball_filter: EnabledLayer<layers::BallFilter, Ground>,
    obstacle_filter: EnabledLayer<layers::ObstacleFilter, Ground>,
    voronoi_cells: EnabledLayer<layers::VoronoiCell, Field>,
}

impl Panel for MapPanel {
    fn is_querying(&self) -> bool {
        self.observations
            .as_ref()
            .is_ok_and(MapObservation::is_querying)
            || self.ball_search_heatmap.is_querying()
            || self.path_obstacles.is_querying()
            || self.obstacles.is_querying()
            || self.path.is_querying()
            || self.odometry.is_querying()
            || self.ball_percepts.is_querying()
            || self.ball_position.is_querying()
            || self.ball_filter.is_querying()
            || self.obstacle_filter.is_querying()
            || self.voronoi_cells.is_querying()
    }

    const STORAGE_ID: &'static str = "map";
    const DISPLAY_NAME: &'static str = "Map";
    const ICON: &'static str = egui_material_icons::icons::ICON_MAP.codepoint;

    fn new(context: PanelCreationContext) -> Self {
        let field = EnabledLayer::new(context.backend.clone(), context.value, true);
        let ball_search_heatmap = EnabledLayer::new(context.backend.clone(), context.value, false);
        let path_obstacles = EnabledLayer::new(context.backend.clone(), context.value, false);
        let obstacles = EnabledLayer::new(context.backend.clone(), context.value, false);
        let path = EnabledLayer::new(context.backend.clone(), context.value, false);
        let robot_pose = EnabledLayer::new(context.backend.clone(), context.value, true);
        let odometry = EnabledLayer::new(context.backend.clone(), context.value, false);
        let ball_percepts = EnabledLayer::new(context.backend.clone(), context.value, false);
        let ball_position = EnabledLayer::new(context.backend.clone(), context.value, true);
        let ball_filter = EnabledLayer::new(context.backend.clone(), context.value, false);
        let obstacle_filter = EnabledLayer::new(context.backend.clone(), context.value, false);
        let voronoi_cells = EnabledLayer::new(context.backend.clone(), context.value, false);

        let observations = MapData::observe(&context).map_err(|error| error.to_string());

        let current_plot_type = context
            .value
            .and_then(|value| value.get("current_plot_type"))
            .and_then(|value| serde_json::from_value::<PlotType>(value.clone()).ok())
            .unwrap_or(PlotType::Ground);
        let zoom_and_pan = context
            .value
            .and_then(|value| value.get("zoom_and_pan"))
            .and_then(|value| serde_json::from_value::<ZoomAndPanTransform>(value.clone()).ok())
            .unwrap_or_default();

        Self {
            current_plot_type,
            observations,
            data: MapData::default(),
            zoom_and_pan,
            field,
            ball_search_heatmap,
            path_obstacles,
            obstacles,
            path,
            robot_pose,
            odometry,
            ball_percepts,
            ball_position,
            ball_filter,
            obstacle_filter,
            voronoi_cells,
        }
    }

    fn save(&self) -> Value {
        json!({
            "current_plot_type": self.current_plot_type,
            "zoom_and_pan": serde_json::to_value(&self.zoom_and_pan).expect("failed to serialize zoom_and_pan"),

            "field": self.field.save(),
            "ball_search_heatmap": self.ball_search_heatmap.save(),
            "path_obstacles": self.path_obstacles.save(),
            "obstacles": self.obstacles.save(),
            "path": self.path.save(),
            "robot_pose": self.robot_pose.save(),
            "odometry": self.odometry.save(),
            "ball_percepts": self.ball_percepts.save(),
            "ball_position": self.ball_position.save(),
            "ball_filter": self.ball_filter.save(),
            "obstacle_filter": self.obstacle_filter.save(),
            "voronoi_cells": self.voronoi_cells.save(),
        })
    }

    fn header_ui(&mut self, ui: &mut Ui, _context: PanelUiContext<'_>) {
        Popup::menu(&ui.button("Overlays"))
            .close_behavior(PopupCloseBehavior::CloseOnClickOutside)
            .show(|ui| {
                self.field.checkbox(ui);
                self.ball_search_heatmap.checkbox(ui);
                self.path_obstacles.checkbox(ui);
                self.obstacles.checkbox(ui);
                self.path.checkbox(ui);
                self.robot_pose.checkbox(ui);
                self.odometry.checkbox(ui);
                self.ball_percepts.checkbox(ui);
                self.ball_position.checkbox(ui);
                self.ball_filter.checkbox(ui);
                self.obstacle_filter.checkbox(ui);
                self.voronoi_cells.checkbox(ui);
            });
        ComboBox::from_id_salt("plot_type_selector")
            .selected_text(format!("{:?}", self.current_plot_type))
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut self.current_plot_type, PlotType::Ground, "Ground");
                ui.selectable_value(&mut self.current_plot_type, PlotType::Field, "Field");
            });
    }

    fn ui(&mut self, ui: &mut Ui, context: PanelUiContext<'_>) {
        let binding_changed = self
            .observations
            .as_ref()
            .is_ok_and(|observation| !observation.is_bound(&context));
        if binding_changed {
            self.data = MapData::default();
        }
        if binding_changed || self.observations.is_err() {
            self.observations = MapData::observe(&context).map_err(|error| error.to_string());
        }
        let result = match &mut self.observations {
            Ok(observation) => observation.update(&mut self.data, MapData::from_frame),
            Err(error) => Err(color_eyre::eyre::eyre!("{error}")),
        };
        if let Err(error) = result {
            ui.colored_label(ui.visuals().error_fg_color, error.to_string());
        }
        let field_dimensions = match self.data.field_dimensions {
            Some(dimensions) => dimensions,
            None => {
                ui.label("no response for field dimensions");
                return;
            }
        };

        let field_visible =
            self.current_plot_type == PlotType::Field || self.data.ground_to_field.is_some();
        let ground_visible =
            self.current_plot_type == PlotType::Ground || self.data.ground_to_field.is_some();
        // An unlocalized ground view still has a valid local origin, but no field pose.
        let ground_to_field = self.data.ground_to_field.unwrap_or_default();
        let (response, mut painter) = match self.current_plot_type {
            PlotType::Field => {
                let width = field_dimensions.width;
                let length = field_dimensions.length;
                let border = field_dimensions.border_strip_width;

                TwixPainter::allocate(
                    ui,
                    vector![2.0 * border + length, 2.0 * border + width],
                    point![
                        border + field_dimensions.length / 2.0,
                        -border - field_dimensions.width / 2.0
                    ],
                    Orientation::RightHanded,
                )
            }
            PlotType::Ground => {
                let (response, painter) = TwixPainter::allocate(
                    ui,
                    vector![2.0, 2.0],
                    point![1.0, -1.0],
                    Orientation::RightHanded,
                );
                (response, painter.transform_painter(ground_to_field))
            }
        };
        self.zoom_and_pan.apply(ui, &mut painter, &response);

        // draw largest layers first so they don't obscure smaller ones
        if field_visible {
            self.field
                .generic_paint(&painter, ground_to_field, &field_dimensions);
            self.ball_search_heatmap
                .generic_paint(&painter, ground_to_field, &field_dimensions);
        }
        if ground_visible {
            self.path_obstacles
                .generic_paint(&painter, ground_to_field, &field_dimensions);
            self.obstacles
                .generic_paint(&painter, ground_to_field, &field_dimensions);
            self.path
                .generic_paint(&painter, ground_to_field, &field_dimensions);
            self.robot_pose
                .generic_paint(&painter, ground_to_field, &field_dimensions);
        }
        if field_visible {
            self.odometry
                .generic_paint(&painter, ground_to_field, &field_dimensions);
        }
        if ground_visible {
            self.ball_percepts
                .generic_paint(&painter, ground_to_field, &field_dimensions);
        }
        if field_visible {
            self.ball_position
                .generic_paint(&painter, ground_to_field, &field_dimensions);
        }
        if ground_visible {
            self.ball_filter
                .generic_paint(&painter, ground_to_field, &field_dimensions);
            self.obstacle_filter
                .generic_paint(&painter, ground_to_field, &field_dimensions);
        }
        if field_visible {
            self.voronoi_cells
                .generic_paint(&painter, ground_to_field, &field_dimensions);
        }
    }
}

fn ground_to_field_at(
    estimate: &LocalizationEstimate,
    status: &LocalizationStatus,
    ground: &TimeWrapper<Option<Isometry3<Ground, Robot>>>,
) -> Option<Isometry2<Ground, Field>> {
    if (estimate.epoch, estimate.generation) != (status.epoch, status.generation)
        || estimate.time.abs_diff(ground.time) > Duration::from_millis(100)
    {
        return None;
    }
    Some(ground_to_field_from_field_to_robot(
        Isometry3::wrap(estimate.robot_to_field?.pose.inner.cast::<f32>().inverse()),
        ground.inner?.inverse(),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_z::time::Time;
    use types::localization::{LocalizationState, PoseEstimate};

    #[test]
    fn replay_pose_uses_recorded_ground_transform_and_checks_freshness() {
        let estimate = LocalizationEstimate {
            time: Time::from_nanos(1_000_000_000),
            epoch: 1,
            generation: 2,
            robot_to_local: PoseEstimate {
                pose: Isometry3::identity(),
                covariance: Default::default(),
            },
            robot_to_field: Some(PoseEstimate {
                pose: Isometry3::wrap(nalgebra::Isometry3::translation(3.0, 2.0, 0.5)),
                covariance: Default::default(),
            }),
        };
        let mut status = LocalizationStatus {
            time: estimate.time,
            epoch: 1,
            generation: 2,
            state: LocalizationState::Tracking,
            heading: None,
        };
        let mut ground = TimeWrapper {
            time: estimate.time,
            inner: Some(Isometry3::wrap(nalgebra::Isometry3::translation(
                0.2, 0.0, -0.5,
            ))),
        };
        let pose = ground_to_field_at(&estimate, &status, &ground).unwrap();
        assert!((pose.translation().x() - 3.2).abs() < 1e-6);
        assert!((pose.translation().y() - 2.0).abs() < 1e-6);
        status.generation = 3;
        assert!(ground_to_field_at(&estimate, &status, &ground).is_none());
        status.generation = 2;
        ground.time = Time::zero();
        assert!(ground_to_field_at(&estimate, &status, &ground).is_none());
        ground.time = estimate.time;
        ground.inner = None;
        assert!(ground_to_field_at(&estimate, &status, &ground).is_none());
        ground.inner = Some(Isometry3::identity());
        let estimate = LocalizationEstimate {
            robot_to_field: None,
            ..estimate
        };
        assert!(ground_to_field_at(&estimate, &status, &ground).is_none());
    }
}
