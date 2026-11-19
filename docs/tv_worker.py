"""Browser TV adapter sharing the desktop steering, sweep and lap solvers."""
import math
import re

import lab_formats

from suspension_geometry_lab.project import rack_travel_bounds, steering_settings
from suspension_geometry_lab.steering import solve_steering
from suspension_geometry_lab.steering_chart import steering_chart, steering_sweep_chart
from suspension_geometry_lab.tv_analysis import calculate_tv_sweep, steering_camber_metrics
from suspension_geometry_lab.tv_lap import calculate_tv_lap_overlay

PLOTS = {
    "top_view": "Steering mechanism (top view)", "steer": "Left / right wheel steering",
    "ackermann": "Ackermann percentage", "camber": "Left / right camber",
    "camber_gain": "Left / right camber gain (deg/mm)",
    "steering_camber_gain": "Steering camber gain (deg/deg)",
}
NOTE = "X forward, Y left, Z up. Positive rack travel moves left. Camber is positive top-out. Rack stroke limits apply to both slider and numeric input."
ROWS = (
    ("rack_travel_mm", "Rack travel (mm)", False),
    ("steer_deg", "Wheel steering (deg)", True),
    ("angle_difference_deg", "Left / right angle difference (deg)", False),
    ("ackermann_pct", "Ackermann (%)", False),
    ("camber_gain_deg_per_mm", "Camber gain (deg/mm)", True),
    ("camber_deg", "Steered camber (deg)", True),
    ("camber_change_deg", "Steering camber change (deg)", True),
    ("kingpin_deg", "Kingpin rotation (deg)", True),
    ("steering_camber_gain_deg_per_deg", "Steering camber gain (deg/deg)", True),
    ("left_minus_right_deg", "Left minus right steering (deg)", False),
    ("ideal_outer_deg", "Ideal outer wheel steering (deg)", False),
    ("tie_rod_z", "Solved upright tie rod Z (m)", False),
    ("inner_corner", "Inner wheel", False), ("outer_corner", "Outer wheel", False),
)
TEXT = {
    "轉向機構": "Steering mechanism", "前軸": "Front axle", "後軸": "Rear axle",
    "車身中心線": "Vehicle centerline", "零位參考": "Neutral reference", "左輪": "Left wheel",
    "右輪": "Right wheel", "主銷軸": "Kingpin", "轉向臂": "Steering arm",
    "主銷轉軸": "Kingpin pivot", "外接點": "upright pickup", "端點": "end",
    "目前阿克曼率未定義": "Current Ackermann is undefined",
    "目前姿態無有效解": "Current pose is unreachable", "目前": "Current",
    "理想阿克曼": "Ideal Ackermann", "阿克曼率": "Ackermann", "位移": "travel",
    "左右輪胎轉角": "Wheel steering", "輪胎轉角": "Wheel steering",
    "左右輪轉向": "Steering", "左右輪": "Left / right", "轉向": "Steering",
    "上視圖": "Top view", "車身橫向": "lateral", "左正右負": "left positive, right negative",
}


def translate(value):
    if isinstance(value, str):
        for original, english in TEXT.items():
            value = value.replace(original, english)
        return value
    if isinstance(value, list):
        return [translate(item) for item in value]
    if isinstance(value, dict):
        return {key: translate(item) for key, item in value.items()}
    return value


def steering_error(error):
    message = str(error)
    if match := re.fullmatch(r"Rack 位移 (.+) mm 超出允許範圍 (.+) ～ (.+) mm。", message):
        return f"Rack travel {match[1]} mm is outside the allowed range {match[2]} to {match[3]} mm."
    if match := re.fullmatch(r"([FR][LR]) 的拉桿在齒條行程 (.+) mm 時無法閉合。", message):
        return f"{match[1]} tie rod cannot close at rack travel {match[2]} mm."
    if match := re.fullmatch(r"([FR][LR]) 無法從齒條零位連續移到 (.+) mm，中途有死點或無解區間。", message):
        return f"{match[1]} cannot reach {match[2]} mm continuously from neutral because of a toggle or unreachable interval."
    return lab_formats.english_error(message)


class TVAnalysis:
    def __init__(self, engine):
        self.engine = engine
        self.sweeps = {}
        self.overlays = {}

    def sweep(self, axle):
        if axle not in self.sweeps:
            self.sweeps[axle] = calculate_tv_sweep(self.engine, axle, steering_settings(self.engine.project)[axle])
        return self.sweeps[axle]

    def result(self, payload, lap=None):
        axle, plot = payload.get("axle", "front"), payload.get("plot", "top_view")
        if axle not in ("front", "rear") or plot not in PLOTS:
            raise ValueError("Choose a valid TV axle and chart.")
        settings = steering_settings(self.engine.project)[axle]
        enabled = axle == "front" or settings["enabled"]
        low, high = rack_travel_bounds(settings) if enabled else (0., 0.)
        travel = payload.get("values", [0.])[0]
        if type(travel) not in (int, float) or not math.isfinite(travel):
            raise ValueError("Enter a finite rack travel in mm.")
        if not enabled:
            travel = 0.
        controls = ([{"label": "Rack travel (mm)", "min": low, "max": high, "value": travel,
                      "disabled": low == high}] if enabled else [])
        pose = state = None
        error = None
        try:
            pose = solve_steering(self.engine, axle, settings, travel_mm=travel)
            metrics = {"axle": axle, **settings, **pose["metrics"],
                       **steering_camber_metrics(self.engine, axle, pose)}
            state = {"metrics": metrics, "points": {}}
        except (ValueError, ArithmeticError) as exc:
            error = steering_error(exc)
        sweep = self.sweep(axle) if plot != "top_view" else None
        overlay = None
        if sweep is not None and lap is not None and payload.get("showLap"):
            key = axle, id(lap)
            if key not in self.overlays:
                self.overlays[key] = calculate_tv_lap_overlay(self.engine, axle, lap, sweep)
            overlay = self.overlays[key]
        spec = (steering_chart(pose) if pose else None) if plot == "top_view" else steering_sweep_chart(
            axle, plot, sweep, metrics=state["metrics"] if state else None, lap=overlay)
        spec = translate(spec)
        if spec:
            spec["title"] = f"{axle.title()} axle · {PLOTS[plot]}"
        note = "" if enabled else "Rear steering is disabled. Rack travel is fixed at 0 mm."
        if overlay is not None:
            note = (f"LapSim matched {overlay['valid']:,} / {overlay['total']:,} samples. "
                    "Rack travel is estimated from the recorded left/right wheel angles.")
            if not overlay["rows"]:
                note += " No matching wheel-angle samples in the configured rack range."
            if plot in ("camber", "camber_gain", "steering_camber_gain"):
                note += " Camber and gains are interpolated static model values at that rack travel."
        return {"state": state, "controls": controls, "error": error, "spec": spec, "tvNote": note,
                "tvRows": [{"key": key, "label": label, "wheel": wheel} for key, label, wheel in ROWS],
                "coverage": {key: sweep[key] for key in ("valid", "total")} if sweep else None}
