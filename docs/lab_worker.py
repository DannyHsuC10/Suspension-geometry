"""Web Worker adapter for the desktop suspension geometry and analysis solvers."""

from dataclasses import fields, is_dataclass
import json
import math
from pathlib import Path
import re
from tempfile import TemporaryDirectory
from time import monotonic

import numpy as np

import lab_formats
from suspension_geometry_lab import analysis, hardpoints
from suspension_geometry_lab.caster_search import search_front_caster
from suspension_geometry_lab.engine import Unreachable
from suspension_geometry_lab.fvsa_search import search_fvsa
from suspension_geometry_lab.input_limits import AXLE_LIMITS
from suspension_geometry_lab.lap_data import LAP_PAGES, calculate_lap_camber, calculate_lap_metrics, read_lap_data
from suspension_geometry_lab.project import FIELDS, travel_limits
from suspension_geometry_lab.roll_center_search import SearchSettings, search_balanced_rear_roll_center, search_roll_center
from suspension_geometry_lab.svsa_search import SVSASearchSettings, search_svsa


FV_PLOTS = {
    "motion": "Front-view mechanism",
    "rc_map": "Roll center position map (x′–y′)",
    "rc_y_prime_mm": "Roll center height surface (y′)",
    "rc_x_prime_mm": "Roll center lateral position surface (x′)",
    "camber_left_deg": "Left camber surface",
    "camber_right_deg": "Right camber surface",
    "camber_gain_left_deg_per_mm": "Left camber gain surface",
    "camber_gain_right_deg_per_mm": "Right camber gain surface",
}
PLOT_LABELS = {
    "front": FV_PLOTS,
    "rear": FV_PLOTS,
    "sv": {
        "motion": "Side-view mechanism",
        "caster_front_deg": "Front caster surface",
        "caster_rear_deg": "Rear caster surface",
        "anti_brake_front_pct": "Front brake-weighted anti surface",
        "anti_brake_rear_pct": "Rear brake-weighted anti surface",
        "anti_drive_front_pct": "Front drive-weighted anti surface",
        "anti_drive_rear_pct": "Rear drive-weighted anti surface",
        "pc_y_prime_mm": "Pitch center height surface (y′)",
        "pc_x_prime_mm": "Pitch center longitudinal position surface (x′)",
        "pc_map": "Pitch center position map (x′–y′)",
    },
    "we": {
        "motion": "3D wheel-end steering",
        "caster_surface": "Wheel-axis tilt by steering and caster",
        "radius": "Contact and material radii",
        "xyz": "Contact coordinates vs. steering",
        "path": "3D contact-point path",
        "tilt": "Caster and wheel tilt vs. steering",
        "arms": "Steering moment arms (Fx, Fy, Fz)",
    },
    "static": {
        "iso": "Full vehicle 3D",
        "front": "Full vehicle front view",
        "side": "Full vehicle side view",
        "top": "Full vehicle top view",
    },
}
PAGE_LABELS = {"front": "Front FV", "rear": "Rear FV", "sv": "Side View", "we": "Wheel End 3D", "static": "Static Vehicle"}
NOTES = {
    "front": "The contact frame starts at the midpoint between the tire contacts. x′ runs left to right and y′ points up. RC0 is the zero-travel roll center, fixed to the body. Camber gain is the local derivative in deg/mm.",
    "rear": "The rear axle uses its own design. The mechanism and roll center use the current contact frame. RC0 is the zero-travel roll center, fixed to the body.",
    "sv": "Caster and mechanical trail use the current contact frame. Anti is positive toward the opposite axle and is weighted by the brake or drive force share. The side-view solver uses fixed arm tangents.",
    "we": "Steering rotates around the static 3D kingpin. X points forward, Y left and Z up. The contact point is the lowest point of the loaded-radius center circle. The material point follows the wheel as it turns.",
    "static": "The vehicle frame is in meters. X points forward, Y left and Z up. Anti is positive toward the opposite axle and is weighted by the applicable axle force share.",
}
HARDPOINT_NOTE = "In hardpoint mode, FV uses equivalent pivots and SV uses fixed tangents from the hardpoint axes. Static 3D and WE3D use the original XYZ coordinates."
CAMBER_STAGES = {"before": "Static camber + body pose", "after": "Body pose + suspension", "steered": "Body pose + suspension + steering"}
CAMBER_LABELS = {f"{side}_camber_{stage}_deg": f"{side.title()} wheel · {label}"
                 for side in ("inner", "outer") for stage, label in CAMBER_STAGES.items()}
CAMBER_PLOT = analysis.LAP_CAMBER_PLOT[0]
CAMBER_PLOT_LABEL = "LapSim cornering camber distribution"

METRIC_LABELS = {
    "heave_mm": "Heave (mm)", "roll_deg": "Roll (deg)", "pitch_deg": "Pitch (deg)",
    "steer_deg": "Steering (deg)", "static_camber_deg": "Static camber (deg)", "wheelbase_mm": "Wheelbase (mm)",
    "cg_height_mm": "CG height (mm)", "wireframe_lines": "Wireframe lines", "named_points": "Named points",
    **{f"travel_{side}_mm": f"{side.title()} wheel travel (mm)" for side in ("left", "right", "front", "rear")},
    **{f"camber_{side}_deg": f"{side.title()} camber (deg)" for side in ("left", "right")},
    **{f"camber_gain_{side}_deg_per_mm": f"{side.title()} camber gain (deg/mm)" for side in ("left", "right")},
    **{f"ride_height_{axle}_mm": f"{axle.title()} ride height (mm)" for axle in ("front", "rear")},
    **{f"svsa_{axle}_mm": f"{axle.title()} SVSA (mm)" for axle in ("front", "rear")},
    **{f"{name}_{axis}_prime_mm": f"{name.upper()} contact-frame {axis}′ (mm)" for name in ("cg", "rc", "pc") for axis in "xy"},
    **{f"{name}_world_{axis}_mm": f"{name.upper()} body-frame {axis.upper()} (mm)" for name in ("cg", "rc", "pc") for axis in "xz"},
    "anti_front_pct": "Front axle anti (%)", "anti_rear_pct": "Rear axle anti (%)",
    "anti_brake_front_pct": "Front brake-weighted anti (%)", "anti_brake_rear_pct": "Rear brake-weighted anti (%)",
    "anti_drive_front_pct": "Front drive-weighted anti (%)", "anti_drive_rear_pct": "Rear drive-weighted anti (%)",
    "caster_front_deg": "Front caster (deg)", "caster_rear_deg": "Rear caster (deg)",
    "trail_front_mm": "Front mechanical trail (mm)", "trail_rear_mm": "Rear mechanical trail (mm)",
    "contact_radius_mm": "Contact point radius (mm)", "material_radius_mm": "Material point radius (mm)",
    **{f"{point}_{axis}_mm": f"{point.title()} {axis.upper()} (mm)" for point in ("contact", "material", "center") for axis in "xyz"},
    "caster_deg": "Caster (deg)", "wheel_tilt_deg": "Wheel-axis tilt (deg)",
    **{f"moment_arm_f{axis}_mm": f"F{axis} steering moment arm (mm)" for axis in "xyz"},
}
CHART_TEXT = {
    "目前姿態無有效解": "The current pose has no valid solution.",
    "目前座標無有效定義": "The current coordinates are undefined.",
    "目前位置超出懸吊行程範圍": "The current position is outside the suspension travel range.",
    "掃描範圍沒有有效座標": "The sweep contains no valid coordinates.",
    "目前位置超出掃描範圍": "The current position is outside the sweep range.",
    "目前數值無有限定義（N/A）": "The current value is undefined (N/A).",
    "沒有可比較的彎道樣本": "No comparable cornering samples are available.",
    "樣本不足 2 筆": "At least two samples are required.",
    "標準差為 0": "Standard deviation is zero.",
    "目前位置": "Current position",
    "前／後行程": "Front/rear travel",
    "目前 ": "Current ",
}
ACTION_ERRORS = {
    "CSV 有重複的車身姿態或轉彎方向欄位。": "The CSV has duplicate body-pose or turn-direction columns.",
    "CSV 有重複的前輪轉向角欄位。": "The CSV has duplicate front-wheel steering columns.",
    "CSV 沒有圈速模擬資料。": "The CSV has no lap simulation data.",
    "輪端圈速 Camber 需要前軸及 CSV 的左右前輪轉角。": "Wheel End 3D lap camber requires both front-wheel steering columns.",
    "Caster 最佳化需要 curvature_1_m 或 ay_g 以辨識外側輪。": "Caster optimization needs curvature_1_m or ay_g to identify the outer wheel.",
    "Caster 最佳化至少需要兩筆彎道樣本，才能建立常態分布。": "Caster optimization needs at least two cornering samples to fit a normal distribution.",
    "候選的有效彎道樣本集合改變，無法公平比較。": "Effective cornering samples differ between candidates.",
    "外側輪 Camber 含無效數值。": "Outer-wheel camber includes an invalid value.",
    "CSV 的四輪行程與車身姿態必須完整且為有限數值。": "The CSV must contain finite travel and body-pose values for every sample.",
    "賽道姿態產生無效的 Camber。": "The lap pose produced an invalid camber angle.",
    "最大允許 Camber 須為 0～90 度的有限數值。": "Maximum camber must be a finite angle from 0 to 90 degrees.",
    "SVSA 搜尋的三個設定都必須是有限數值。": "All SVSA search settings must be finite numbers.",
    "前後軸各需選擇一個 Caster 絕對變化量下限或上限。": "Choose one lower or upper caster change limit per axle.",
    "Caster 絕對變化量的下限／上限不得為負。": "Caster change limits must be zero or positive.",
    "目標姿態的 Caster 無有效數值。": "Caster is undefined at the target pose.",
    "前後軸必須都找到合格值，才會一起套用。": "Both axles need a candidate that meets the caster change limits.",
    "組合姿態的 Caster 變化量未達設定條件。": "Caster change for the combined pose is outside the requested limits.",
    "搜尋設定必須是有限數值。": "Search settings must be finite numbers.",
    "上壓／下垂行程各須為 0～200 mm，且不可同時為 0。": "Bump and droop must each be 0 to 200 mm and provide a nonzero total travel range.",
    "高度範圍內沒有符合 RC 設定級距的候選值。": "The interval contains no candidate on the roll center design grid.",
    "行程內存在不可達姿態": "The travel range contains an unreachable pose.",
    "行程內瞬心無有限交點": "The instant center has no finite intersection within the travel range.",
    "行程內 A 臂退化": "An A-arm becomes degenerate within the travel range.",
    "行程內瞬心通過平行奇異位置": "The instant center crosses a parallel singularity within the travel range.",
    "接地點／瞬心無法定義有效力線": "A contact point and instant center fail to define a force line.",
    "行程內 RC 力線平行或跨越奇異位置": "Roll center force lines are parallel or cross a singularity within the travel range.",
    "行程內 RC 無有限位置": "The roll center has no finite position within the travel range.",
    "RC 觸地、低於地面或離地餘裕不足": "Roll center clearance is below the configured minimum.",
    "A 臂車身 X 超出候選幾何的有效範圍": "Chassis mount X is outside the candidate geometry range.",
    "A 臂車身 X 超出解析解幾何的有效範圍。": "Chassis mount X is outside the analytical solution geometry range.",
    "零行程 RC 無有限交點，無法計算載荷轉移平衡。": "The zero-travel roll center has no finite intersection for the load-transfer calculation.",
    "解析式分母無效，無法求得唯一的有限後軸 RC。": "The analytical equation has no unique finite rear roll center.",
    "合格": "Pass",
    "通過": "Pass",
    "Caster 變化量未達條件": "Caster change is outside the requested limits.",
}

_engine = None
_lap = None
_lap_name = None
_mapping_cache = {}
_lap_cache = {}
_last_pose = None


def view_fields(page):
    shared = ["wheelbase", "cg_height", "ground_clearance", "free_radius", "tire_stiffness", "brake_bias_front", "drive_force_front"]
    if page == "front":
        shared = ["track_front", "free_radius", "tire_width"]
    elif page == "rear":
        shared = ["track_front", "track_rear", "wheelbase", "cg_to_front", "cg_height", "roll_stiffness_ratio", "free_radius", "tire_width"]
    elif page == "static":
        shared = list(FIELDS["vehicle"])
    groups = [{"group": "vehicle", "label": "Vehicle and wheel settings", "keys": shared}]
    for axle in ((page,) if page in ("front", "rear") else ("front", "rear")):
        if page in ("front", "rear"):
            keys = ["fvsa", "rc_height", "static_camber_deg", "kpi_deg", "scrub_radius", "upper_offset", "lower_offset", "lower_mount_x", "upper_mount_x"]
        elif page == "sv":
            keys = ["caster_deg", "mechanical_trail", "anti", "svsa", "upper_offset", "lower_offset"]
        elif page == "we":
            keys = ["kpi_deg", "caster_deg", "scrub_radius", "mechanical_trail", "static_camber_deg"]
        else:
            keys = list(FIELDS[axle])
        groups.append({"group": axle, "label": f"{axle.title()} axle design", "keys": keys})
    groups.append({"group": "analysis", "label": "Sweep range", "keys": list(FIELDS["analysis"])})
    return groups


def view_metadata():
    return {page: {"label": PAGE_LABELS[page], "note": NOTES[page],
                   "plots": [{"key": key, "label": PLOT_LABELS[page][key]} for key, _ in analysis.PLOTS[page]],
                   "fields": view_fields(page), "lap": page in (*LAP_PAGES, "we"),
                   "lapOverlay": page in LAP_PAGES,
                   "camberSeries": [{"key": key, "label": CAMBER_LABELS[key]}
                                    for key, *_ in analysis.LAP_CAMBER_SERIES_BY_PAGE.get(page, ())]}
            for page in PAGE_LABELS}


def lap_descriptor():
    return {"name": _lap_name, "total": len(_lap.travels) if _lap else 0,
            "camber": bool(_lap and _lap.poses), "steering": bool(_lap and _lap.front_steer_deg)}


def project_summary():
    return {**lab_formats.project_response(_engine), "lap": lap_descriptor()}


def install_project(project):
    global _engine, _last_pose
    _engine = lab_formats.validated_engine(project)
    _last_pose = None
    _mapping_cache.clear()
    _lap_cache.clear()
    return project_summary()


def pose_args(payload):
    page = payload.get("page", "front")
    mode = payload.get("mode", "travel")
    axle = payload.get("axle", "front")
    plot = payload.get("plot", "iso" if page == "static" else "motion")
    if page not in PLOT_LABELS or mode not in ("travel", "pose") or axle not in ("front", "rear"):
        raise ValueError("Choose a valid view, pose mode and axle.")
    if plot == CAMBER_PLOT and page in analysis.LAP_CAMBER_SERIES_BY_PAGE:
        if not _lap or not _lap.poses or (page == "we" and not _lap.front_steer_deg):
            raise ValueError("Import LapSim CSV with body pose and turn direction. Wheel End 3D also requires both front-wheel steering angles.")
        if page == "we":
            axle = "front"
    elif plot not in PLOT_LABELS[page]:
        raise ValueError("Choose a chart for the selected view.")
    return page, mode, axle, plot


def pose_controls(page, mode, plot, inputs):
    if page == "static":
        return []
    low, high = travel_limits(_engine.project)
    if page == "we":
        extent = _engine.project["analysis"]["steer_deg"]
        label = "Front steering from LapSim CSV" if plot == CAMBER_PLOT else "Steering (deg)"
        limits = [(label, -extent, extent)]
    elif mode == "pose":
        heave = min(high, max(low, inputs[0]))
        if page == "sv":
            extent = _engine.pitch_angle_limit()
            angle = ("Pitch (deg)", -extent, extent)
        else:
            try:
                lower_roll, upper_roll = _engine.fv[page].roll_bounds(heave)
            except Unreachable:
                lower_roll = upper_roll = 0.
            angle = ("Roll (deg)", lower_roll, upper_roll)
        limits = [("Heave (mm)", low, high), angle]
    else:
        ends = ("Front", "Rear") if page == "sv" else ("Left", "Right")
        limits = [(f"{end} wheel travel (mm)", low, high) for end in ends]
    return [{"label": label, "min": low, "max": high, "value": min(high, max(low, value)),
             "disabled": low == high or (page == "we" and plot == CAMBER_PLOT)}
            for (label, low, high), value in zip(limits, inputs)]


def english_error(error):
    message = str(error)
    if message in ACTION_ERRORS:
        return ACTION_ERRORS[message]
    columns_prefix = "缺少或重複的相對位移欄位："
    if message.startswith(columns_prefix):
        columns = message.removeprefix(columns_prefix)
        return "Missing or duplicate relative travel columns: " + columns.replace("、", ", ") + "."
    if match := re.fullmatch(r"CSV 第 (\d+) 行的 (\w+) (不是數字|須為有限值)。", message):
        return f"CSV line {match[1]}: {match[2]} must be a finite number."
    if message.startswith("FVSA 搜尋需要四輪相對位移"):
        return "FVSA optimization needs four wheel travel fields, heave_mm, roll_deg, pitch_deg and either curvature_1_m or ay_g."
    if message.startswith("Caster 最佳化需要 CSV 每筆資料完整包含"):
        return "Caster optimization needs steer_FL_deg and steer_FR_deg in every CSV row."
    if match := re.fullmatch(r"輪心行程 (.+) mm 對應的接地行程超出設定或無解。", message):
        return f"Wheel-center travel of {match[1]} mm requires an out-of-range or unreachable contact travel."
    if match := re.fullmatch(r"RC 搜尋高度須介於 (.+)～(.+) mm，且下限不得大於上限。", message):
        return f"Choose an ordered roll center search interval between {match[1]} and {match[2]} mm."
    if match := re.fullmatch(r"(解析解|取整後)設計高度 (.+) m 超出允許界線 (.+)～(.+) m。", message):
        kind = "Analytical" if match[1] == "解析解" else "Rounded"
        return f"{kind} design height of {match[2]} m falls outside the allowed range of {match[3]} to {match[4]} m."
    prefixes = {
        "固定前軸未通過全行程檢查：": "The fixed front axle failed the full-travel check. ",
        "解析解未通過後軸全行程檢查：": "The rear analytical design failed the full-travel check. ",
        "解析解的幾何／平衡模型無效：": "The analytical design failed the geometry or load-transfer check. ",
        "前後軸組合驗證失敗：": "The combined front and rear design did not pass validation. ",
        "靜態幾何無解：": "Static geometry has no solution. ",
        "外側輪無法建立常態分布：": "Unable to fit the outer-wheel normal distribution. ",
    }
    for prefix, label in prefixes.items():
        if message.startswith(prefix):
            reason = message.removeprefix(prefix).split("。請先調整")[0]
            reason = reason.rstrip("。") if reason.rstrip("。") in ACTION_ERRORS else reason
            return label + english_error(reason)
    return CHART_TEXT.get(message) or lab_formats.english_error(message)


def solve(payload):
    global _last_pose
    page, mode, axle, plot = pose_args(payload)
    values = list(payload.get("values", []))[:2]
    if not all(type(value) in (int, float) and math.isfinite(value) for value in values):
        raise ValueError("Enter finite pose values.")
    controls = pose_controls(page, mode, plot, values + [0.] * (2 - len(values)))
    inputs = tuple(control["value"] for control in controls)
    key = page, mode, axle, inputs
    if _last_pose and _last_pose[0] == key:
        _, state, error = _last_pose
    else:
        state, error = None, None
        try:
            if page in _engine.fv:
                solver = _engine.fv[page]
                state = (solver.pose if mode == "pose" else solver.solve)(*inputs)
            elif page == "sv":
                state = (_engine.sv_pose if mode == "pose" else _engine.solve_sv)(*inputs)
            elif page == "we":
                state = _engine.solve_we(inputs[0], axle)
            else:
                state = {"metrics": {"wireframe_lines": len(_engine.scene.lines),
                                     "named_points": len(analysis.static_point_rows(_engine)),
                                     **_engine.static_anti_metrics}, "points": {}}
        except Unreachable as exc:
            error = english_error(exc)
        _last_pose = key, state, error
    return {"state": state, "controls": controls, "error": error}


def get_mapping(page, mode, axle, plot):
    if page == "static":
        return None
    if plot == CAMBER_PLOT:
        key = (page, CAMBER_PLOT)
        if key not in _lap_cache:
            _lap_cache[key] = calculate_lap_camber(_engine, "rear" if page == "rear" else "front", _lap,
                                                   include_steering=page == "we")
        return _lap_cache[key]
    key = (page, mode, axle if page == "we" else None)
    if key not in _mapping_cache:
        _mapping_cache[key] = analysis.calculate_mapping(_engine, page, mode, axle)
    return _mapping_cache[key]


def get_overlay(page):
    if page not in _lap_cache:
        _lap_cache[page] = calculate_lap_metrics(_engine, page, _lap.travels)
    return _lap_cache[page]


def mapping_coverage(mapping):
    return {key: mapping[key] for key in ("valid", "total", "straight") if key in mapping} if mapping else None


def english_chart_text(value):
    for original, english in CHART_TEXT.items():
        value = value.replace(original, english)
    return re.sub(r"([A-Za-z_]+_(?:mm|deg|pct))(?=:|$)",
                  lambda match: METRIC_LABELS.get(match[1], match[1]), value)


def format_plot(spec, page, plot, mapping, payload):
    if spec is None:
        return
    if plot == CAMBER_PLOT:
        axle = "Rear" if page == "rear" else "Front"
        spec["title"] = f"{axle} cornering camber · {mapping['valid']:,} / {mapping['total']:,} samples"
        spec["xlabel"] = "Camber (deg) · 0.25° bins · positive is top-out"
        spec["ylabel"] = "Sample count"
        names = {name: CAMBER_LABELS[key] for key, name, *_ in analysis.LAP_CAMBER_SERIES_BY_PAGE[page]}
        for series in spec.get("series", []) + spec.get("normal_curves", []):
            series["name"] = names[series["name"]]
            if "reason" in series:
                series["reason"] = english_chart_text(series["reason"])
        spec["show_normal_curves"] = payload.get("showNormalCurves", True)
        spec["hidden_series"] = [CAMBER_LABELS.get(name, name) for name in payload.get("hiddenSeries", [])]
    else:
        spec["title"] = PLOT_LABELS[page][plot]
        if plot == "motion" and page != "we":
            spec["title"] = f"{PAGE_LABELS[page]} mechanism · Current contact frame"
        if plot.startswith("anti_"):
            spec["zlabel"] = METRIC_LABELS[plot]
        for series in spec.get("series", []):
            series["name"] = METRIC_LABELS.get(series["name"], series["name"])
    if "current_status" in spec:
        spec["current_status"] = english_chart_text(spec["current_status"])
    for marker in spec.get("markers", []):
        marker["name"] = english_chart_text(marker["name"])
        if "label_lines" in marker:
            marker["label_lines"] = [english_chart_text(line) for line in marker["label_lines"]]
    if not payload.get("construction", True) and page != "static" and "world_lines" in spec:
        spec["world_lines"] = [line for line in spec["world_lines"] if not line["construction"]]


def empty_sweep(spec):
    if "mesh" in spec:
        points = (point for row in spec["mesh"] for point in row)
    elif "series" in spec:
        points = (point for item in spec["series"] for point in zip(item["x"], item["y"]))
    else:
        return False
    return not any(all(value is not None and math.isfinite(value) for value in point) for point in points)


def plot_result(payload):
    page, mode, axle, plot = pose_args(payload)
    result = solve(payload)
    if plot == "motion" and not result["state"]:
        return {**result, "spec": None, "coverage": None}
    mapping = get_mapping(page, mode, axle, plot) if plot != "motion" else None
    overlay = (get_overlay(page) if _lap and payload.get("showLap", False)
               and page in LAP_PAGES and plot not in ("motion", CAMBER_PLOT) else None)
    spec = analysis.make_plot(page, plot, result["state"], mapping, _engine, payload.get("groups"),
                              show_static_references=payload.get("staticReferences", True), lap=overlay)
    format_plot(spec, page, plot, mapping, payload)
    if spec and plot != CAMBER_PLOT and empty_sweep(spec):
        spec = {"title": spec["title"], "current_status": "The sweep contains no plottable results for this chart."}
    result.update(spec=spec, coverage=mapping_coverage(mapping))
    if plot == CAMBER_PLOT:
        rows = mapping["rows"]
        result["camberStats"] = [{"key": key, "label": CAMBER_LABELS[key], "count": len(rows),
                                  "min": min((row[key] for row in rows), default=None),
                                  "max": max((row[key] for row in rows), default=None)}
                                 for key, *_ in analysis.LAP_CAMBER_SERIES_BY_PAGE[page]]
    return result


def import_lap(payload):
    global _lap, _lap_name
    with TemporaryDirectory() as directory:
        path = Path(directory) / "lap.csv"
        path.write_text(payload["text"], encoding="utf-8", newline="")
        candidate = read_lap_data(path)
    _lap = candidate
    _lap_name = re.split(r"[/\\]", payload.get("name") or "LapSim CSV")[-1]
    _lap_cache.clear()
    return lap_descriptor()


def clear_lap():
    global _lap, _lap_name
    _lap = _lap_name = None
    _lap_cache.clear()
    return lap_descriptor()


def search_progress(kind):
    callback = globals().get("publish_progress")
    last = 0.

    def notify(*args):
        nonlocal last
        if not callback:
            return
        axle, values = (args[0], args[1:]) if kind == "svsa" else (None, args)
        tested, total, candidate, *details = values
        now = monotonic()
        if tested in (1, total) or now - last >= .075:
            last = now
            reason = details[-1] if details and isinstance(details[-1], str) else ""
            callback(json.dumps({"kind": kind, "axle": axle, "tested": tested, "total": total,
                                 "candidate": candidate, "reason": english_error(reason) if reason else ""}))

    return notify


def english_search_result(value, key=None):
    if is_dataclass(value):
        value = {field.name: getattr(value, field.name) for field in fields(value)}
    if isinstance(value, dict):
        if key == "rejected":
            return [{"reason": english_error(reason), "count": count} for reason, count in value.items()]
        return {name: english_search_result(item, name) for name, item in value.items()}
    if isinstance(value, str) and value and key in ("reason", "error", "failure_reason"):
        return english_error(value)
    return value


def search(payload):
    kind = payload["kind"]
    page = payload.get("page", "front")
    project = payload.get("project", _engine.project)
    settings = payload.get("settings", {})
    if hardpoints.enabled(project):
        raise ValueError("Optimization uses parameter-mode design inputs. Switch to parameter mode to search.")
    progress = search_progress(kind)
    target_view = None
    status = "No design meets the search conditions."
    if kind == "roll-center":
        if page not in ("front", "rear"):
            raise ValueError("Roll center optimization is available on Front FV and Rear FV.")
        ranges = project["analysis"]
        options = SearchSettings(round(settings.get("lower_m", -.005) * 1000, 9), AXLE_LIMITS["rc_height"].high * 1000,
                                 ranges["droop_mm"], ranges["bump_mm"])
        if page == "rear":
            result = search_balanced_rear_roll_center(project, options, progress=progress)
        else:
            result = search_roll_center(project, page, options, progress=progress)
        if result.project:
            left, right = result.check.worst_travel_mm
            status = (f"The best {page} roll center design height is {result.project[page]['rc_height']:.3f} m. "
                      f"The lowest roll center is {result.check.min_height_mm:.3f} mm at {left:+g} / {right:+g} mm of "
                      f"left / right travel. The largest lateral offset is {result.check.max_abs_x_mm:.3f} mm. "
                      f"Each design check covered {result.travel_samples**2:,} travel combinations per axle.")
            if result.balance:
                status += (f" The analytical rear design height before rounding is {result.analytic.rear_design_m:.6g} m. "
                           f"The front load-transfer share after rounding is {result.balance.front_transfer_fraction:.4%} "
                           f"({result.balance.error*100:+.4f} percentage points from 50%). "
                           f"The lowest front roll center is {result.front_check.min_height_mm:.3f} mm.")
        elif result.failure_reason:
            status = english_error(result.failure_reason)
        elif result.rejected:
            reason = max(result.rejected, key=result.rejected.get)
            status = f"Checked {result.tested} roll center heights. Most frequent rejection: {english_error(reason)}"
    elif kind == "fvsa":
        if _lap is None:
            raise ValueError("Import LapSim CSV with body pose and turn direction before optimizing FVSA.")
        result = search_fvsa(project, page, _lap, settings.get("max_camber_deg", 2.), progress=progress)
        if result.project:
            status = (f"The best {page} FVSA is {result.project[page]['fvsa']:.5g} m. "
                      f"Absolute camber peaks at {result.peak.peak_deg:.5g}° in sample {result.peak.sample} "
                      f"({result.peak.wheel}, {result.peak.signed_deg:+.5g}°).")
        if result.stopped_peak:
            status += (f" The design at {result.stopped_fvsa_m:.5g} m reached "
                       f"{result.stopped_peak.peak_deg:.5g}° absolute camber and exceeded the limit.")
        elif result.error:
            status += f" Search stopped at {result.stopped_fvsa_m:.5g} m. {english_error(result.error)}"
        elif result.stop_reason == "upper_limit":
            status += " The search reached the upper design limit."
    elif kind == "svsa":
        options = SVSASearchSettings.from_bounds(
            settings.get("heave_mm", 7.),
            settings.get("front_bound", "min"), settings.get("front_limit_deg", settings.get("front_min_deg", .15)),
            settings.get("rear_bound", "max"), settings.get("rear_limit_deg", settings.get("rear_max_deg", .1)),
        )
        result = search_svsa(project, options, progress=progress)
        front_bound, front_limit = options.criterion("front")
        rear_bound, rear_limit = options.criterion("rear")
        signs = {"min": "≥", "max": "≤"}
        limits = (f"Front |ΔCaster| {signs[front_bound]} {front_limit:g}°. "
                  f"Rear |ΔCaster| {signs[rear_bound]} {rear_limit:g}°.")
        if result.project:
            target_view = {"page": "sv", "mode": "pose", "values": [options.heave_mm, 0.]}
            front, rear = result.front.best, result.rear.best
            status = (f"At {options.heave_mm:g} mm heave, the best SVSA is {front.svsa_m:g} m front "
                      f"(caster {front.zero_deg:.6g}° to {front.target_deg:.6g}°, change {front.signed_deg:+.6f}°), "
                      f"rear {rear.svsa_m:g} m (caster {rear.zero_deg:.6g}° to {rear.target_deg:.6g}°, "
                      f"change {rear.signed_deg:+.6f}°). Checked {result.front.tested} front and {result.rear.tested} rear lengths.")
        elif result.error:
            status = english_error(result.error)
        status = f"{status} {limits}"
    elif kind == "caster":
        if _lap is None:
            raise ValueError("Import LapSim CSV with body pose and both front-wheel steering angles before optimizing caster.")
        result = search_front_caster(project, _lap, progress=progress)
        if result.best:
            target_view = {"page": "we", "axle": "front", "plot": CAMBER_PLOT}
            best = result.best
            status = (f"The best front caster is {best.caster_deg:g}°. Outer-wheel camber averages {best.mean_deg:+.6f}° "
                      f"with standard deviation {best.std_deg:.6f}°. Observed range {best.minimum_deg:+.6f}° to "
                      f"{best.maximum_deg:+.6f}°. Used {best.samples:,} of {result.mapping['total']:,} cornering samples "
                      f"and excluded {result.mapping['straight']:,} straight-line samples. Checked {result.tested} designs.")
            if result.before:
                status += f" Initial caster {result.before.caster_deg:g}° gave a camber mean of {result.before.mean_deg:+.6f}°."
        elif result.rejected:
            reason = max(result.rejected, key=result.rejected.get)
            status = f"Checked {result.tested} caster angles. Most frequent rejection: {english_error(reason)}"
    else:
        raise ValueError("Choose a valid optimization.")
    details = english_search_result({field.name: getattr(result, field.name) for field in fields(result)
                                     if field.name not in ("project", "mapping")})
    if kind == "caster":
        details["mapping"] = mapping_coverage(result.mapping)
    if kind == "svsa":
        for axle in ("front", "rear"):
            change = getattr(result, axle).best
            if change:
                details[axle]["best"].update(signed_deg=change.signed_deg, absolute_deg=change.absolute_deg)
    return {"project": result.project, "result": details, "status": status, "view": target_view}


def handle(action, payload):
    if action.startswith("project-"):
        return lab_formats.handle(action, payload)
    if action == "init":
        if _engine is None:
            install_project(lab_formats.handle("project-default", {}))
        return {**project_summary(), "views": view_metadata(), "metricLabels": METRIC_LABELS,
                "camberPlot": {"key": CAMBER_PLOT, "label": CAMBER_PLOT_LABEL}, "hardpointNote": HARDPOINT_NOTE}
    if action == "project":
        return install_project(payload["project"])
    if _engine is None:
        raise ValueError("Initialize the geometry engine first.")
    if action == "solve":
        return solve(payload)
    if action == "plot":
        return plot_result(payload)
    if action == "lap_import":
        return import_lap(payload)
    if action == "lap_clear":
        return clear_lap()
    if action == "search":
        return search(payload)
    if action == "export-data":
        page, mode, axle, plot = pose_args(payload)
        pose = solve(payload)
        kind = payload.get("kind")
        if pose["error"] and kind in ("state", "points", "step"):
            raise ValueError(pose["error"])
        needs_mapping = kind == "mapping" or (kind == "report" and plot != "motion")
        mapping = get_mapping(page, mode, axle, plot) if needs_mapping else None
        payload = {**payload, "page": page, "plot": plot, "mode": mode, "axle": axle,
                   "values": [control["value"] for control in pose["controls"]]}
        if plot == CAMBER_PLOT:
            payload["lapName"] = _lap_name
        return lab_formats.handle(action, payload, engine=_engine, state=pose["state"], mapping=mapping)
    raise ValueError("Unknown geometry worker action.")


def json_value(value):
    """Normalize NumPy values and undefined numbers before returning to JavaScript."""
    if isinstance(value, np.ndarray):
        value = value.tolist()
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, dict):
        return {key: json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_value(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def dispatch(action, payload_json):
    """One JSON request/response boundary for Pyodide, also usable by native Python."""
    try:
        response = {"ok": True, "data": json_value(handle(action, json.loads(payload_json)))}
    except Exception as exc:
        response = {"ok": False, "error": english_error(str(exc) or type(exc).__name__)}
    return json.dumps(response, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
