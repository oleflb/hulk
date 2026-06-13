#!/usr/bin/env python3
#
# Run with:
#   uv run --script crates/localization-factrs/scripts/visualize_trajectory_plotly.py
#
# uv run --script crates/localization-factrs/scripts/visualize_trajectory_plotly.py \
#               /tmp/graph_trajectory.json
#
# /// script
# requires-python = ">=3.11"
# dependencies = ["click>=8.1", "dash>=2.18", "plotly>=5.24"]
# ///
"""Serve localization-factrs solver output with Plotly Dash."""

from __future__ import annotations

import json
from pathlib import Path

import click
from dash import Dash, dcc, html
import plotly.graph_objects as go


COLORS = {
    "ground_truth": "#0072B2",
    "optimized": "#D55E00",
    "raw_backend_solves": "#009E73",
    "propagated_solves": "#CC79A7",
    "landmarks": "#F0E442",
}


def trajectory_from_samples(samples):
    return [
        {
            "t": sample["timestamp_seconds"],
            "x": sample["position"][0],
            "y": sample["position"][1],
            "z": sample["position"][2],
        }
        for sample in samples
    ]


def ground_truth_from_simulation(data):
    return [
        {
            "t": measurement["timestamp_seconds"],
            "x": measurement["ground_truth_pose"]["position"][0],
            "y": measurement["ground_truth_pose"]["position"][1],
            "z": measurement["ground_truth_pose"]["position"][2],
        }
        for measurement in data.get("measurements", [])
    ]


def landmarks_from_output(data):
    return [
        {
            "name": landmark["name"],
            "x": landmark["position"][0],
            "y": landmark["position"][1],
            "z": landmark["position"][2],
        }
        for landmark in data.get("landmarks", [])
    ]


def landmarks_from_simulation(data):
    return [
        {"name": name, "x": position[0], "y": position[1], "z": position[2]}
        for name, position in data.get("landmark_global_positions", {}).items()
    ]


def build_trajectories(data, graph_data=None):
    output = graph_data if graph_data is not None else data
    trajectories = []

    if "ground_truth" in output:
        ground_truth = trajectory_from_samples(output.get("ground_truth", []))
    else:
        ground_truth = ground_truth_from_simulation(data)
    if ground_truth:
        trajectories.append(
            {
                "name": "ground truth",
                "trajectory": ground_truth,
                "color": COLORS["ground_truth"],
                "width": 6,
                "dash": None,
                "mode": "lines",
                "marker_size": None,
                "legendgroup": "ground truth",
            }
        )

    optional_streams = [
        (
            "optimized",
            "optimized live/frontend",
            COLORS["optimized"],
            6,
            None,
            "lines",
            None,
        ),
        (
            "raw_backend_solves",
            "raw backend solves",
            COLORS["raw_backend_solves"],
            4,
            "dot",
            "lines+markers",
            4,
        ),
        (
            "propagated_solves",
            "propagated solve states",
            COLORS["propagated_solves"],
            4,
            "dash",
            "lines+markers",
            4,
        ),
    ]
    for key, name, color, width, dash, mode, marker_size in optional_streams:
        trajectory = trajectory_from_samples(output.get(key, []))
        if not trajectory:
            continue
        trajectories.append(
            {
                "name": name,
                "trajectory": trajectory,
                "color": color,
                "width": width,
                "dash": dash,
                "mode": mode,
                "marker_size": marker_size,
                "legendgroup": name,
            }
        )

    landmark_points = landmarks_from_output(output) or landmarks_from_simulation(data)

    return trajectories, landmark_points


def axis_lists(trajectory):
    return (
        [sample["x"] for sample in trajectory],
        [sample["y"] for sample in trajectory],
        [sample["z"] for sample in trajectory],
        [sample["t"] for sample in trajectory],
    )


def nearest_by_time(trajectory, time):
    if not trajectory:
        return None
    return min(trajectory, key=lambda sample: abs(sample["t"] - time))


def trajectory_prefix(trajectory, time):
    prefix = [sample for sample in trajectory if sample["t"] <= time]
    if prefix:
        return prefix

    nearest = nearest_by_time(trajectory, time)
    return [nearest] if nearest is not None else []


def trajectory_trace(
    name,
    trajectory,
    color,
    width=5,
    dash=None,
    legendgroup=None,
    mode="lines",
    marker_size=None,
):
    x, y, z, t = axis_lists(trajectory)
    line = {"color": color, "width": width}
    if dash is not None:
        line["dash"] = dash
    marker = {"color": color}
    if marker_size is not None:
        marker["size"] = marker_size

    return go.Scatter3d(
        name=name,
        x=x,
        y=y,
        z=z,
        mode=mode,
        line=line,
        marker=marker,
        customdata=t,
        legendgroup=legendgroup,
        hovertemplate=(
            f"{name}<br>"
            "t=%{customdata:.3f}s<br>"
            "x=%{x:.3f}<br>"
            "y=%{y:.3f}<br>"
            "z=%{z:.3f}<extra></extra>"
        ),
    )


def trajectory_trace_from_spec(spec, trajectory=None):
    return trajectory_trace(
        spec["name"],
        spec["trajectory"] if trajectory is None else trajectory,
        spec["color"],
        width=spec["width"],
        dash=spec["dash"],
        legendgroup=spec["legendgroup"],
        mode=spec["mode"],
        marker_size=spec["marker_size"],
    )


def current_marker_trace(name, sample, color, legendgroup):
    if sample is None:
        x = []
        y = []
        z = []
        customdata = []
    else:
        x = [sample["x"]]
        y = [sample["y"]]
        z = [sample["z"]]
        customdata = [sample["t"]]

    return go.Scatter3d(
        name=f"current {name}",
        x=x,
        y=y,
        z=z,
        mode="markers",
        marker={"color": color, "size": 8, "line": {"color": "#222222", "width": 2}},
        customdata=customdata,
        showlegend=False,
        legendgroup=legendgroup,
        hovertemplate=(
            f"current {name}<br>"
            "t=%{customdata:.3f}s<br>"
            "x=%{x:.3f}<br>"
            "y=%{y:.3f}<br>"
            "z=%{z:.3f}<extra></extra>"
        ),
    )


def landmark_trace(landmark_points):
    return go.Scatter3d(
        name="landmarks",
        x=[landmark["x"] for landmark in landmark_points],
        y=[landmark["y"] for landmark in landmark_points],
        z=[landmark["z"] for landmark in landmark_points],
        text=[landmark["name"] for landmark in landmark_points],
        mode="markers+text",
        marker={"color": COLORS["landmarks"], "size": 6, "line": {"color": "#3d3d3d", "width": 1}},
        textposition="top center",
        hovertemplate="%{text}<br>x=%{x:.3f}<br>y=%{y:.3f}<br>z=%{z:.3f}<extra></extra>",
    )


def scene_ranges(*point_sets, padding_fraction=0.05):
    points = [point for point_set in point_sets for point in point_set]
    if not points:
        return {}

    raw_ranges = {axis: raw_range([point[axis] for point in points]) for axis in ("x", "y", "z")}
    return {
        axis: padded_range(axis_range, padding_fraction)
        for axis, axis_range in raw_ranges.items()
    }


def raw_range(values):
    return min(values), max(values)


def padded_range(axis_range, padding_fraction, min_span=1.0):
    minimum, maximum = axis_range
    center = (minimum + maximum) / 2.0
    span = max(maximum - minimum, min_span)
    return centered_range((center, center), span * (1.0 + 2.0 * padding_fraction))


def centered_range(axis_range, span):
    center = (axis_range[0] + axis_range[1]) / 2.0
    half_span = span / 2.0
    return [center - half_span, center + half_span]


def scene_aspect_ratio(ranges):
    if not ranges:
        return {"x": 1, "y": 1, "z": 1}

    spans = {
        axis: axis_range[1] - axis_range[0]
        for axis, axis_range in ranges.items()
    }
    max_span = max(spans.values())
    if max_span == 0.0:
        return {"x": 1, "y": 1, "z": 1}

    return {axis: span / max_span for axis, span in spans.items()}


def scene_axis(title, axis_range):
    axis = {
        "title": title,
        "showgrid": True,
        "gridcolor": "#9a9a9a",
        "gridwidth": 2,
        "zeroline": True,
        "zerolinecolor": "#6f6f6f",
        "zerolinewidth": 3,
        "showbackground": True,
        "backgroundcolor": "#f4f4f4",
    }
    if axis_range is not None:
        axis["range"] = axis_range
        axis["autorange"] = False
    return axis


def build_figure(data, graph_data=None, slider_frames=180):
    trajectories, landmark_points = build_trajectories(data, graph_data)
    ranges = scene_ranges(
        *[spec["trajectory"] for spec in trajectories],
        landmark_points,
    )

    fig = go.Figure(
        data=[trajectory_trace_from_spec(spec) for spec in trajectories]
        + [landmark_trace(landmark_points)]
        + [
            current_marker_trace(
                spec["name"],
                spec["trajectory"][-1] if spec["trajectory"] else None,
                spec["color"],
                spec["legendgroup"],
            )
            for spec in trajectories
        ]
    )
    add_time_slider(fig, trajectories, slider_frames)
    add_bounds_menu(fig, trajectories, landmark_points)

    fig.update_layout(
        title="Localization Trajectory",
        template="plotly_white",
        height=900,
        legend={
            "title": {"text": "Click traces to hide/show"},
            "orientation": "h",
            "yanchor": "bottom",
            "y": 1.08,
            "xanchor": "right",
            "x": 1.0,
            "groupclick": "togglegroup",
        },
        margin={"l": 8, "r": 8, "t": 110, "b": 150},
        scene={
            "xaxis": scene_axis("x", ranges.get("x")),
            "yaxis": scene_axis("y", ranges.get("y")),
            "zaxis": scene_axis("z", ranges.get("z")),
            "aspectmode": "manual",
            "aspectratio": scene_aspect_ratio(ranges),
            "camera": {"eye": {"x": 1.45, "y": -1.85, "z": 1.15}},
        },
    )
    return fig


def add_time_slider(fig, trajectories, slider_frame_count):
    times = sorted(
        {
            sample["t"]
            for spec in trajectories
            for sample in spec["trajectory"]
        }
    )
    if not times:
        return

    current_marker_offset = len(trajectories) + 1
    animated_trace_indices = list(range(len(trajectories))) + list(
        range(current_marker_offset, current_marker_offset + len(trajectories))
    )
    time_indices = slider_indices(len(times), slider_frame_count)

    frames = []
    steps = []
    for frame_index, time_index in enumerate(time_indices):
        time = times[time_index]
        frame_name = str(frame_index)
        frames.append(
            go.Frame(
                name=frame_name,
                traces=animated_trace_indices,
                data=[
                    trajectory_trace_from_spec(
                        spec,
                        trajectory_prefix(spec["trajectory"], time),
                    )
                    for spec in trajectories
                ]
                + [
                    current_marker_trace(
                        spec["name"],
                        nearest_by_time(spec["trajectory"], time),
                        spec["color"],
                        spec["legendgroup"],
                    )
                    for spec in trajectories
                ],
            )
        )
        steps.append(
            {
                "label": f"{time:.2f}s",
                "method": "animate",
                "args": [
                    [frame_name],
                    {
                        "mode": "immediate",
                        "frame": {"duration": 0, "redraw": True},
                        "transition": {"duration": 0},
                    },
                ],
            }
        )

    fig.frames = frames
    final_frame = str(len(time_indices) - 1)
    fig.update_layout(
        sliders=[
            {
                "active": len(time_indices) - 1,
                "currentvalue": {"prefix": "time: ", "font": {"size": 14}},
                "pad": {"t": 55, "b": 10},
                "steps": steps,
            }
        ],
        updatemenus=list(fig.layout.updatemenus)
        + [
            {
                "type": "buttons",
                "showactive": False,
                "direction": "left",
                "x": 0.0,
                "y": -0.04,
                "xanchor": "left",
                "yanchor": "top",
                "buttons": [
                    {
                        "label": "Play",
                        "method": "animate",
                        "args": [
                            None,
                            {
                                "frame": {"duration": 35, "redraw": True},
                                "fromcurrent": True,
                                "transition": {"duration": 0},
                            },
                        ],
                    },
                    {
                        "label": "Pause",
                        "method": "animate",
                        "args": [
                            [None],
                            {
                                "mode": "immediate",
                                "frame": {"duration": 0, "redraw": False},
                                "transition": {"duration": 0},
                            },
                        ],
                    },
                    {
                        "label": "Show all",
                        "method": "animate",
                        "args": [
                            [final_frame],
                            {
                                "mode": "immediate",
                                "frame": {"duration": 0, "redraw": True},
                                "transition": {"duration": 0},
                            },
                        ],
                    },
                ],
            }
        ],
    )


def add_bounds_menu(fig, trajectories, landmark_points):
    buttons = []
    all_ranges = scene_ranges(
        *[spec["trajectory"] for spec in trajectories],
        landmark_points,
    )
    if all_ranges:
        buttons.append(bounds_button("all data", all_ranges))

    for spec in trajectories:
        ranges = scene_ranges(spec["trajectory"])
        if ranges:
            buttons.append(bounds_button(spec["name"], ranges))

    if not buttons:
        return

    fig.update_layout(
        updatemenus=list(fig.layout.updatemenus)
        + [
            {
                "type": "dropdown",
                "showactive": True,
                "direction": "down",
                "x": 0.32,
                "y": -0.04,
                "xanchor": "left",
                "yanchor": "top",
                "buttons": buttons,
            }
        ]
    )


def bounds_button(label, ranges):
    return {
        "label": f"Bounds: {label}",
        "method": "relayout",
        "args": [scene_relayout(ranges)],
    }


def scene_relayout(ranges):
    return {
        "scene.xaxis.range": ranges.get("x"),
        "scene.yaxis.range": ranges.get("y"),
        "scene.zaxis.range": ranges.get("z"),
        "scene.xaxis.autorange": False,
        "scene.yaxis.autorange": False,
        "scene.zaxis.autorange": False,
        "scene.aspectmode": "manual",
        "scene.aspectratio": scene_aspect_ratio(ranges),
    }


def slider_indices(length, requested_count):
    if length <= 0:
        return []
    count = max(2, min(length, requested_count))
    return sorted({round(index * (length - 1) / (count - 1)) for index in range(count)})


def load_json(path):
    with path.open(encoding="utf-8") as file:
        return json.load(file)


def create_app(figure, source_path, graph_output_path=None):
    app = Dash(__name__)
    app.title = "Localization Trajectory"

    source_description = f"trajectory: {source_path}"
    if graph_output_path is not None:
        source_description += f" | solver output: {graph_output_path}"

    app.layout = html.Main(
        [
            html.H1("Localization Trajectory"),
            html.P(source_description),
            dcc.Graph(
                id="trajectory-graph",
                figure=figure,
                style={"height": "90vh"},
                config={"displaylogo": False, "responsive": True},
            ),
        ],
        style={
            "fontFamily": "system-ui, sans-serif",
            "margin": "0 auto",
            "maxWidth": "1600px",
            "padding": "1rem",
        },
    )

    return app


@click.command(context_settings={"show_default": True})
@click.argument(
    "trajectory",
    required=False,
    default=Path("/tmp/graph_trajectory.json"),
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
)
@click.option(
    "--graph-output",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    help="Optional solver output JSON when the positional input is simulation data.",
)
@click.option("--host", default="127.0.0.1", help="Dash server host.")
@click.option("--port", default=8050, type=int, help="Dash server port.")
@click.option("--debug/--no-debug", default=False, help="Run Dash in debug mode.")
@click.option(
    "--slider-frames",
    default=180,
    type=click.IntRange(min=2),
    help="Number of time slider frames to generate.",
)
def main(trajectory, graph_output, host, port, debug, slider_frames):
    data = load_json(trajectory)
    graph_data = None
    if graph_output is not None:
        graph_data = load_json(graph_output)

    figure = build_figure(data, graph_data=graph_data, slider_frames=slider_frames)
    app = create_app(figure, trajectory, graph_output)
    click.echo(f"Serving trajectory viewer at http://{host}:{port}")
    app.run(host=host, port=port, debug=debug)


if __name__ == "__main__":
    main()
