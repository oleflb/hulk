use std::f32::consts::{FRAC_PI_2, PI, TAU};

use coordinate_systems::Field;
use linear_algebra::Point2;
use types::field_dimensions::{FieldDimensions, Half, Side};

pub(super) fn field_segments(d: &FieldDimensions) -> [[Point2<Field>; 2]; 3] {
    [
        [d.t_crossing(Side::Left), d.t_crossing(Side::Right)],
        [
            d.corner(Half::Own, Side::Left),
            d.corner(Half::Opponent, Side::Left),
        ],
        [
            d.corner(Half::Own, Side::Right),
            d.corner(Half::Opponent, Side::Right),
        ],
    ]
}

pub(super) fn center_circle(d: &FieldDimensions) -> (Point2<Field>, f32, f32, f32) {
    (d.center(), d.center_circle_diameter / 2.0, 0.0, TAU)
}

pub(super) fn half_segments(d: &FieldDimensions, half: Half) -> [[Point2<Field>; 2]; 3] {
    [
        [d.corner(half, Side::Left), d.corner(half, Side::Right)],
        [
            d.goal_box_corner(half, Side::Left),
            d.goal_box_corner(half, Side::Right),
        ],
        [
            d.penalty_box_corner(half, Side::Left),
            d.penalty_box_corner(half, Side::Right),
        ],
    ]
}

pub(super) fn box_segments(d: &FieldDimensions, half: Half, side: Side) -> [[Point2<Field>; 2]; 2] {
    [
        [
            d.goal_box_corner(half, side),
            d.goal_box_goal_line_intersection(half, side),
        ],
        [
            d.penalty_box_corner(half, side),
            d.penalty_box_goal_line_intersection(half, side),
        ],
    ]
}

pub(super) fn corner_arc(
    d: &FieldDimensions,
    half: Half,
    side: Side,
) -> (Point2<Field>, f32, f32, f32) {
    let start = match (half, side) {
        (Half::Own, Side::Left) => -FRAC_PI_2,
        (Half::Own, Side::Right) => 0.0,
        (Half::Opponent, Side::Left) => PI,
        (Half::Opponent, Side::Right) => FRAC_PI_2,
    };
    (
        d.corner(half, side),
        d.corner_arc_radius,
        start,
        start + FRAC_PI_2,
    )
}
