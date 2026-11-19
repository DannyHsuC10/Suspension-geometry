"""Vehicle-coordinate top-view and rack-travel charts for steering geometry."""

import math

import numpy as np


RACK_COLOR = "#2dd4bf"
ROD_COLOR = "#fb923c"
ARM_COLOR = "#c4b5fd"
KINGPIN_COLOR = "#f0abfc"
NEUTRAL_COLOR = "#64748b"
WHEEL_COLORS = ("#38bdf8", "#fbbf24")
ACKERMANN_COLOR = "#a78bfa"
TV_PLOTS = (
    ("top_view", "轉向機構上視圖"),
    ("steer", "左右輪胎轉角"),
    ("ackermann", "阿克曼率"),
    ("camber", "左右輪 Camber"),
    ("camber_gain", "左右輪 Camber gain (°/mm)"),
    ("steering_camber_gain", "左右輪轉向 Camber gain (°/°)"),
)


def _project(point):
    """Put the front of the vehicle at the top and its left at screen left."""
    return [-float(point[1]), float(point[0])]


def _path(name, points, color, dash=()):
    x, y = zip(*(_project(point) for point in points))
    return {"name": name, "x": list(x), "y": list(y), "color": color, "dash": dash}


def _dot(name, point, color):
    return {"name": name, "position": _project(point), "color": color, "radius": 3, "label_lines": []}


def _heading_arrow(wheel):
    center = wheel["center"]
    forward = wheel["forward"]
    left = np.array([-forward[1], forward[0], 0.0])
    half_length = np.max((wheel["tire_outline"] - center) @ forward)
    point = center + 0.73 * half_length * forward
    shoulder = point - 0.19 * half_length * forward
    wing = 0.10 * half_length * left
    return [center, point, shoulder + wing, point, shoulder - wing]


def steering_chart(result):
    """Build an equal-scale LabChart specification from a solved steering pose."""
    wheels = result["wheels"]
    is_front = "FL" in wheels
    left, right = ("FL", "FR") if is_front else ("RL", "RR")
    rack = result["rack_ends"]
    rack_neutral = result["rack_ends_neutral"]
    corners = (left, right)
    neutral_wheels = result["wheels_neutral"]

    x_positions = [float(rack[left][0])]
    for corner in corners:
        x_positions.extend(float(p[0]) for p in wheels[corner]["tire_outline"])
        x_positions.extend(float(p[0]) for p in neutral_wheels[corner]["tire_outline"])
        x_positions.append(float(wheels[corner]["tie_point"][0]))
    x_span = max(x_positions) - min(x_positions)
    margin = 0.05 * x_span
    centerline = [(min(x_positions) - margin, 0), (max(x_positions) + margin, 0)]
    series = [_path("車身中心線", centerline, "#64748b")]
    markers = []
    annotations = []

    series.append(_path("零位參考", [rack_neutral[left], rack_neutral[right]], NEUTRAL_COLOR, (5, 5)))
    for corner in corners:
        series.append(_path("零位參考", neutral_wheels[corner]["tire_outline"], NEUTRAL_COLOR, (5, 5)))

    for corner, color, side in zip(corners, WHEEL_COLORS, ("左輪", "右輪")):
        wheel = wheels[corner]
        tie = wheel["tie_point"]
        axis = wheel["kingpin_axis"]
        origin = wheel["kingpin_origin"]
        pivot = origin + axis * np.dot(tie - origin, axis)
        series.extend([
            _path(f"{side} {corner}", wheel["tire_outline"], color),
            _path(f"{side} {corner}", _heading_arrow(wheel), color),
            _path("主銷軸", [wheel["kingpin_lower"], wheel["kingpin_upper"]], KINGPIN_COLOR, (3, 3)),
            _path("轉向臂", [pivot, tie], ARM_COLOR),
            _path("Tie rod", [tie, rack[corner]], ROD_COLOR),
        ])
        markers.extend([_dot("主銷轉軸", pivot, KINGPIN_COLOR), _dot("Tie rod 外接點", tie, ROD_COLOR),
                        _dot("Rack 端點", rack[corner], RACK_COLOR)])
        annotations.append({"name": corner, "position": _project(wheel["center"]), "color": color,
                            "offset": [-24 if corner == left else 11, -13]})

    series.append(_path("Rack", [rack[left], rack[right]], RACK_COLOR))
    axle_label = "前軸" if is_front else "後軸"
    return {"title": f"{axle_label}轉向機構 · TV 上視圖", "xlabel": "-Y [m] · 車身橫向",
            "ylabel": "X [m]", "equal": True,
            "series": series, "markers": markers, "annotations": annotations}


def _finite(value):
    return value is not None and math.isfinite(value)


def steering_sweep_chart(axle, plot, sweep, *, metrics=None, lap=None):
    """Plot solved TV sweeps, the current pose and measured LapSim samples."""
    if plot not in dict(TV_PLOTS) or plot == "top_view":
        raise ValueError(f"Unrecognized steering sweep plot: {plot}")
    axle_label, prefix = ("前軸", "F") if axle == "front" else ("後軸", "R")
    if plot == "ackermann":
        signals = (("ackermann_pct", "阿克曼率", ACKERMANN_COLOR),)
        ylabel, unit = "阿克曼率 [%]", "%"
    else:
        field, ylabel, unit = {
            "steer": ("steer_deg", "輪胎轉角 [°]", "°"),
            "camber": ("camber_deg", "Camber [°]", "°"),
            "camber_gain": ("camber_gain_deg_per_mm", "Camber gain [°/mm]", "°/mm"),
            "steering_camber_gain": ("steering_camber_gain_deg_per_deg", "轉向 Camber gain [°/°]", "°/°"),
        }[plot]
        signals = ((f"left_{field}", f"左輪 {prefix}L", WHEEL_COLORS[0]),
                   (f"right_{field}", f"右輪 {prefix}R", WHEEL_COLORS[1]))

    rows = sweep["rows"]
    x = [float(row["rack_travel_mm"]) for row in rows]
    series = [{"name": label, "x": x, "y": [row.get(key) for row in rows], "color": color}
              for key, label, color in signals]
    if plot == "ackermann" and x:
        series.append({"name": "理想阿克曼 100%", "x": [x[0], x[-1]], "y": [100., 100.],
                       "color": NEUTRAL_COLOR, "dash": (5, 5)})

    markers = []
    metrics = metrics or {}
    current = metrics.get("rack_travel_mm")
    if _finite(current):
        for index, (key, label, color) in enumerate(signals):
            value = metrics.get(key)
            if not _finite(value):
                continue
            labels = [f"{label} {value:.4g}{unit}"]
            if not markers:
                labels.insert(0, f"目前 Rack {current:.4g} mm")
            markers.append({"name": f"目前 {label}", "position": [float(current), float(value)],
                            "radius": 5, "color": color, "label_lines": labels,
                            "offset": [12, -38 if index == 0 else 14]})

    spec = {"title": f"{axle_label} · {dict(TV_PLOTS)[plot]}",
            "xlabel": "Rack 位移 [mm]（左正右負）", "ylabel": ylabel,
            "series": series, "markers": markers, "space_callouts": True}
    if not markers:
        spec["current_status"] = ("目前阿克曼率未定義" if plot == "ackermann" and _finite(current)
                                  else "目前姿態無有效解")
    if lap is not None:
        points, colors, visible = [], [], 0
        for row in lap["rows"]:
            position = row.get("rack_travel_mm")
            if not _finite(position):
                continue
            previous = len(points)
            for key, _, color in signals:
                value = row.get(key)
                if _finite(value):
                    points.append([float(position), float(value)])
                    colors.append("#f8fafc" if plot == "ackermann" else color)
            visible += len(points) > previous
        spec.update(lap_points=points, lap_colors=colors, lap_total=lap["total"], lap_visible=visible)

    # Include the configured stroke limits even where the kinematics fail.
    y_reference = next((value for curve in series for value in curve["y"] if _finite(value)), 0.)
    spec["fit_points"] = [[position, y_reference] for position in (x[0], x[-1])] if x else []
    spec["fit_points"].extend(spec.get("lap_points", ()))
    return spec
