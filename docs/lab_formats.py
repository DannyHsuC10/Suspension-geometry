"""Browser project editing and downloads backed by the desktop geometry engine."""

import base64
import copy
import csv
import html
import io
import json
import math
from datetime import datetime, timedelta, timezone
from pathlib import Path
import re
from tempfile import TemporaryDirectory

from suspension_geometry_lab import hardpoints
from suspension_geometry_lab.analysis import calculate_mapping, static_point_rows
from suspension_geometry_lab.engine import Engine
from suspension_geometry_lab.export import STEP_PAGES, point_rows, write_step
from suspension_geometry_lab.input_limits import FIELD_LIMITS, INPUT_STEPS, round_design_input
from suspension_geometry_lab.project import DERIVED_AXLE_FIELDS, FIELDS, default_project, mount_x_settings
from suspension_geometry_lab.project_import import MissingField, fill_import_project, read_import_project


GROUP_NAMES = {"vehicle": "Vehicle and wheels", "front": "Front axle", "rear": "Rear axle", "analysis": "Sweep ranges"}
PAGE_NAMES = {"front": "Front FV", "rear": "Rear FV", "sv": "Side View (SV)", "we": "Wheel End (WE3D)", "static": "Static Full Car"}
FIELD_LABELS = {
    "wheelbase": "Wheelbase", "cg_to_front": "CG to front axle", "track_front": "Front track",
    "track_rear": "Rear track", "cg_height": "CG height", "mass": "Total mass", "gravity": "Gravity",
    "ground_clearance": "Ground clearance", "free_radius": "Unloaded tire radius",
    "tire_stiffness": "Tire vertical stiffness", "tire_width": "Tire width",
    "rim_inner_diameter": "Rim inner diameter", "rim_gap": "Rim clearance",
    "clearance_steer_deg": "Steering angle for clearance", "brake_bias_front": "Front brake fraction",
    "drive_force_front": "Front drive fraction", "roll_stiffness_ratio": "Front / rear roll stiffness ratio",
    "fvsa": "Front-view swing arm (FVSA)", "rc_height": "Roll center height", "kpi_deg": "Kingpin inclination",
    "scrub_radius": "Scrub radius", "upper_offset": "Upper joint offset from wheel center",
    "lower_offset": "Lower joint offset from wheel center", "lower_mount_x": "Lower arm chassis |Y|",
    "upper_mount_x": "Upper arm chassis |Y|", "caster_deg": "Caster", "mechanical_trail": "Mechanical trail",
    "anti": "Axle anti ratio", "svsa": "Signed side-view swing arm (SVSA)", "static_camber_deg": "Static camber",
    "anti_brake": "Brake-weighted anti ratio", "anti_drive": "Drive-weighted anti ratio",
    "anti_dive": "Anti-dive ratio", "anti_lift": "Anti-lift ratio", "anti_squat": "Anti-squat ratio",
    "bump_mm": "Bump travel", "droop_mm": "Droop travel", "steer_deg": "Steering sweep ±",
    "caster_min_deg": "Minimum design caster", "caster_max_deg": "Maximum design caster",
    "samples": "Samples per axis", "travel_mm": "Symmetric travel", "roll_deg": "Legacy roll range",
    "pitch_deg": "Legacy pitch range", "name": "Project name", "enabled": "Use hardpoint geometry",
}
MOUNT_LABELS = {
    "upper_front_x": "Upper arm front mount X", "upper_rear_x": "Upper arm rear mount X",
    "lower_front_x": "Lower arm front mount X", "lower_rear_x": "Lower arm rear mount X",
}
POINT_LABELS = {
    "UCA_F": "Upper arm front mount", "UCA_R": "Upper arm rear mount", "UBJ": "Upper ball joint",
    "LCA_F": "Lower arm front mount", "LCA_R": "Lower arm rear mount", "LBJ": "Lower ball joint",
}
EXTRA_HINTS = {
    "wheelbase": "Greater than the CG-to-front distance.",
    "cg_to_front": "Less than the wheelbase.", "ground_clearance": "At or below the CG height.",
    "rim_inner_diameter": "Less than the unloaded tire diameter.",
    "rim_gap": "Twice the clearance must be less than the rim inner diameter.",
    "lower_mount_x": "Less than half the axle track.", "upper_mount_x": "Less than half the axle track.",
    "drive_force_front": "0 = rear-wheel drive, 1 = front-wheel drive.",
    "roll_stiffness_ratio": "Use total axle stiffness, including springs and anti-roll bars.",
    "static_camber_deg": "Negative angles tilt the top of the wheel inboard.",
    "anti": "Positive toward the opposite axle.", "svsa": "Positive toward the opposite axle.",
    "bump_mm": "Bump and droop cannot both be zero.", "droop_mm": "Bump and droop cannot both be zero.",
    "caster_min_deg": "Less than the maximum design caster.", "caster_max_deg": "Greater than the minimum design caster.",
}
UNIT_LABELS = {"0..1": "fraction", "1=100%": "fraction", "前/後": "ratio", "5..51": ""}
REPORT_NOTES = {
    "front": "The left-to-right x′ axis follows the current contact line, with the origin midway between the contact points. The upward y′ axis is perpendicular to the contact line. RC0 is the chassis-fixed zero-travel reference.",
    "rear": "The rear axle uses its own geometry. The diagram and roll center are shown in the current road frame. RC0 is the chassis-fixed zero-travel reference.",
    "sv": "Caster and mechanical trail use the current road frame. The unweighted anti ratio is positive toward the opposite axle. Brake- and drive-weighted values apply their respective axle force fractions.",
    "we": "The wheel end rotates around the static kingpin axis. The contact point is the lowest point on the loaded-radius center circle. The material point follows the original contact point on the rigid wheel.",
    "static": "The static vehicle uses body coordinates in metres. X points forward, Y points left, and Z points up.",
}

EXACT_ERRORS = {
    "不支援的專案版本；目前為 schema_version=7，可讀取舊版 1～6。": "Use a SuspensionGeometryLab JSON project with schema version 1–7.",
    "需要 SuspensionGeometryLab 專案（schema_version、name、vehicle、front、rear、analysis）。": "Use a SuspensionGeometryLab JSON project with vehicle, axle and analysis settings.",
    "需要 SuspensionGeometryLab 專案，檔案含未知欄位。": "The JSON contains fields outside the SuspensionGeometryLab project schema.",
    "svsa_design 只適用於版本 5，且需要前後軸各一個布林值。": "svsa_design is supported in version 5 and requires a Boolean for each axle.",
    "專案名稱須為 1..128 個字元，且不可全為空白。": "Enter a nonblank project name of 1–128 characters.",
    "舊專案 SVSA 不得為負。": "The legacy project requires nonnegative SVSA values.",
    "重心必須在軸距內。": "Place the center of gravity between the front and rear axles.",
    "舊專案前煞車比例必須大於 0 且小於 1。": "The legacy project requires a front brake fraction between 0 and 1, exclusive.",
    "前軸加速驅動力比例須為 0..1；0 為後驅、1 為前驅。": "Front drive fraction must be between 0 and 1.",
    "離地高與間隙不得為負。": "Ground clearance and rim clearance must be nonnegative.",
    "車底離地高不得高於重心高度。": "Ground clearance must be at or below the CG height.",
    "輪圈內徑必須小於輪胎自由直徑（2 × 輪胎自由半徑）。": "Rim inner diameter must be less than twice the unloaded tire radius.",
    "輪圈避讓間隙的兩倍必須小於輪圈內徑。": "Twice the rim clearance must be less than the rim inner diameter.",
    "輪胎負載半徑須大於 0；請檢查質量、重心位置與輪胎垂直剛性。": "Loaded tire radius must be positive. Check mass, CG position and tire vertical stiffness.",
    "輪胎負載半徑小於等於 0；檢查質量與輪胎剛性。": "Loaded tire radius must be positive. Check mass and tire vertical stiffness.",
    "靜態避讓轉角須為 0..80 deg。": "Steering angle for clearance must be between 0° and 80°.",
    "KPI 與 caster 須介於 -85 與 85 deg。": "Kingpin inclination and caster must be between −85° and 85°.",
    "上壓、下垂行程各須為 0..200 mm，且不可同時為 0。": "Bump and droop travel must each be between 0 and 200 mm and cannot both be zero.",
    "轉向範圍須大於 0 且不超過 80 deg。": "Steering sweep must be greater than 0° and at most 80°.",
    "Caster 掃描範圍無效。": "Enter an increasing caster sweep range.",
    "網格點數必須為 5..51 的整數。": "Enter a whole number of samples per axis from 5 to 51.",
    "hardpoints 必須包含 enabled 布林值與 corners 硬點。": "hardpoints requires an enabled Boolean and all four corners.",
    "arm_mount_x 需要前後軸各四個 X 設定。": "Supply four arm mount X settings for each axle.",
    "硬點必須包含 FL、FR、RL、RR 四個輪角。": "Hardpoints must include FL, FR, RL and RR.",
    "左側硬點 Y 必須大於 0；右側硬點 Y 必須小於 0。": "Use positive Y for left hardpoints and negative Y for right hardpoints.",
    "未知的輪角或硬點名稱。": "Unknown wheel corner or hardpoint name.",
    "Initial Sizing 匯出檔須為 schema_version 1、2 或 3。": "Use an Initial Sizing JSON export with schema version 1, 2 or 3.",
    "Initial Sizing 的輪胎剛性須為大於 0 的有限數值（N/m）。": "Initial Sizing tire stiffness must be finite and positive in N/m.",
    "兩個接地點重合，無法建立接地座標系。": "Coincident contact points cannot define a road frame.",
    "FV 上臂與羊角無交點。": "The front-view upper arm and upright cannot intersect in this pose.",
    "SV 切線路徑與羊角無交點。": "The side-view tangent path and upright cannot intersect in this pose.",
    "Heave / roll 反算未收斂。": "The heave and roll pose did not converge.",
    "Heave / pitch 反算未收斂。": "The heave and pitch pose did not converge.",
    "目前姿態無解；請調整滑桿後再匯出。": "Adjust the pose to a reachable configuration before exporting.",
    "懸吊管徑必須是大於 0 的有限數值（mm）。": "Enter a finite positive tube diameter in millimetres.",
    "STEP 輪胎半徑與胎寬必須是正的有限數值。": "STEP requires a finite positive tire radius and width.",
    "STEP 匯出適用於前 FV、後 FV、SV 與靜態全車。": "STEP is available for Front FV, Rear FV, Side View and Static Vehicle.",
    "STEP 座標與方向必須是三個有限數值。": "STEP points and directions must contain three finite coordinates.",
    "目前沒有可匯出的 STEP 線框。": "The current pose contains no exportable STEP wireframe.",
}

DYNAMIC_SUFFIXES = {
    "必須是 JSON 物件，且只能包含專案定義的欄位。": "must be a JSON object containing recognized fields.",
    "必須是 JSON 物件。": "must be a JSON object.",
    "必須是有限數值。": "must be a finite number.",
    "須為大於 0 的有限數值（m）。": "must be finite and positive in metres.",
    "必須大於 0。": "must be greater than zero.",
    "欄位不完整或含未知欄位。": "has missing or unrecognized fields.",
    "XYZ 座標須為最多三格的 JSON 陣列。": "needs a JSON array with up to three coordinates.",
    "目前參數無法產生有限車身接點。": "cannot produce a finite chassis mount with these parameters.",
    "需要三個有限 XYZ 座標（m）。": "requires three finite XYZ coordinates.",
    "需要上／下 A 臂各三點，共六個硬點。": "needs all six upper and lower arm hardpoints.",
    "硬點不可越過車身中心線。": "must stay on its side of the vehicle centerline.",
    "左右硬點必須以 Y=0 鏡射對稱。": "must match the opposite side reflected across Y = 0.",
    "前後車身接點重合，無法建立 A 臂旋轉軸。": "has coincident chassis mounts, which cannot define an arm rotation axis.",
    "球接點位於車身旋轉軸上，A 臂三點不可共線。": "has a ball joint on the chassis rotation axis. The three arm hardpoints must be non-collinear.",
    "目前 FV 平面模型要求前後車身接點有非零 X 間距。": "requires nonzero X separation between the front and rear chassis mounts.",
    "側視投影沒有可用的運動方向。": "has no motion direction in the side-view projection.",
    "上球接點 Z 必須高於下球接點。": "requires the upper ball joint to have a greater Z than the lower ball joint.",
    "FVSA、Au、Al 必須大於 0。": "requires positive FVSA and wheel-center offsets.",
    "車身接點 |Y| 不得為負。": "requires a nonnegative chassis mount |Y|.",
    "車身接點必須在半輪距內。": "requires chassis mount |Y| within half the track.",
    "A 臂 X 設定必須包含上下臂各前後兩點。": "must include the front and rear X settings for the upper and lower arms.",
    "A 臂 X 必須是有限數值，或 null 表示使用極限。": "requires finite arm mount X values or null for automatic limits.",
    "舊 anti 設定沒有有限瞬心，無法轉成 SVSA。": "has an undefined legacy instant center and cannot be converted to SVSA.",
    "舊專案的 Anti 力線平行，無法換算有限 SVSA。": "has parallel legacy anti force lines, which cannot define a finite SVSA.",
    "Anti 設定無法產生有效的有限 SVSA。": "could not determine a finite SVSA from the anti setting.",
    "Anti 設定無法產生有限的瞬心高度。": "could not determine a finite instant center height from the anti setting.",
    "STEP 座標必須為有限數值。": "requires finite STEP coordinates.",
}


def english_error(message):
    message = str(message)
    if message in EXACT_ERRORS:
        return EXACT_ERRORS[message]
    if message.isascii():
        return message
    if match := re.fullmatch(r"此姿態需要行程 (.+) mm，超出下垂 (.+) ～ 上壓 (.+) mm。", message):
        return f"This pose needs {match[1]} mm of travel, outside the range of −{match[2]} to {match[3]} mm."
    if match := re.fullmatch(r"要求的行程 (.+) mm 在目前幾何分支不可達。", message):
        return f"The requested travel of {match[1]} mm is unreachable on the current geometry branch."
    if match := re.fullmatch(r"(.+)：瞬心與 (.+) 重合或無有限力線。", message):
        return f"{match[1]}: the force line through {match[2]} and the instant center is undefined."
    if match := re.fullmatch(r"(\w+) · (.+) 允許範圍：.+", message):
        fields = FIELDS.get(match[1], {})
        key = next((key for key, (label, _) in fields.items() if label == match[2]), None)
        if key:
            return f"{GROUP_NAMES[match[1]]} · {FIELD_LABELS[key]}: {field_hint(match[1], key)}"
    for source, translation in DYNAMIC_SUFFIXES.items():
        if message.endswith(source):
            label = message.removesuffix(source).strip(" ：:").removeprefix("舊專案 ").replace("Initial Sizing 的 ", "Initial Sizing ")
            return f"{label} {translation}" if label.isascii() else translation.capitalize()
    if not re.search(r"[\u3400-\u9fff]", message):
        return message
    return "The geometry solver could not use these inputs. Check the project settings and pose limits."


def unit_label(unit):
    return UNIT_LABELS.get(unit, unit)


def finite_limit(value):
    return value if math.isfinite(value) else None


def field_hint(group, key):
    limit = FIELD_LIMITS[group][key]
    unit = unit_label(FIELDS[group][key][1])
    suffix = f"\N{NO-BREAK SPACE}{unit}" if unit and unit not in ("fraction", "ratio") else ""
    if limit.min_abs:
        hint = f"Finite signed value with magnitude ≥ {limit.min_abs:g}{suffix}."
    elif not math.isfinite(limit.low):
        hint = "Any finite value."
    elif not math.isfinite(limit.high):
        operator = ">" if limit.open_low else "≥"
        hint = f"Finite value {operator} {limit.low:g}{suffix}."
    else:
        low = "(" if limit.open_low else "["
        high = ")" if limit.open_high else "]"
        hint = f"{'Whole numbers' if limit.integer else 'Range'} {low}{limit.low:g}, {limit.high:g}{high}{suffix}."
    if key == "anti":
        step = INPUT_STEPS[key]
        increment = f"Step\N{NO-BREAK SPACE}{step:g} ({step * 100:g} percentage points)."
    elif key in INPUT_STEPS:
        increment = f"Step\N{NO-BREAK SPACE}{INPUT_STEPS[key]:g}{suffix}."
    elif limit.integer:
        increment = "Step 1."
    else:
        increment = "Any increment."
    return " ".join(part for part in (hint, increment, EXTRA_HINTS.get(key)) if part)


def fields_metadata():
    return {
        group: [{"key": key, "label": FIELD_LABELS[key], "unit": unit_label(unit), "hint": field_hint(group, key),
                 "minimum": finite_limit(FIELD_LIMITS[group][key].low), "maximum": finite_limit(FIELD_LIMITS[group][key].high),
                 "openMinimum": FIELD_LIMITS[group][key].open_low, "openMaximum": FIELD_LIMITS[group][key].open_high,
                 "integer": FIELD_LIMITS[group][key].integer, "minimumMagnitude": FIELD_LIMITS[group][key].min_abs}
                for key, (_, unit) in fields.items()]
        for group, fields in FIELDS.items()
    }


def default_browser_project():
    project = default_project()
    project["name"] = "Baseline Vehicle"
    return project


def mount_message(check):
    message = check.get("message", "")
    if not message:
        return ""
    if check["limit_min_x"] is None:
        return "The geometry does not provide two finite clearance limits."
    if check["excess_mm"] > 0:
        return f"Outside the clearance range by {check['excess_mm']:.6g} mm. The preview uses the automatic mounts."
    if "前後順序錯誤" in message:
        return "The rear mount X must be less than the front mount X. The preview uses the automatic mounts."
    if "另一個接點超限" in message:
        return "The other mount on this arm is outside its clearance range. The preview uses the automatic mounts."
    return "The arm plane does not define a unique Z at this X and Y. The preview uses the automatic mounts."


def mount_checks(engine):
    return {
        axle: {key: {**check, "message": mount_message(check)} for key, check in checks.items()}
        for axle, checks in engine.scene.geometry.get("mount_position_checks", {}).items()
    }


def validated_engine(project):
    engine = Engine(project)
    if not hardpoints.enabled(engine.project):
        for axle, checks in mount_checks(engine).items():
            for key, check in checks.items():
                if check["requested_x"] is not None and check["fallback"]:
                    raise ValueError(f"{GROUP_NAMES[axle]} · {MOUNT_LABELS[key]}. {check['message']} Adjust the X value or leave it blank for automatic placement.")
    return engine


def project_response(engine):
    return {"project": engine.project, "derived": engine.derived_parameters, "mountChecks": mount_checks(engine)}


def prepare_project(project):
    engine = Engine(project)
    return {
        **project_response(engine), "fields": fields_metadata(), "mountFields": MOUNT_LABELS,
        "pointLabels": POINT_LABELS, "groups": GROUP_NAMES,
        "derivedFields": {key: {"label": FIELD_LABELS[key], "unit": unit_label(unit)}
                          for key, (_, unit) in DERIVED_AXLE_FIELDS.items()},
        "points": engine.project.get("hardpoints", {}).get("corners") or hardpoints.from_scene(engine.scene),
        "mounts": mount_x_settings(engine.project),
    }


def editable_project(payload):
    candidate = copy.deepcopy(payload["project"])
    base = payload.get("base")
    if base:
        for group, fields in FIELDS.items():
            for key in fields:
                value = candidate[group].get(key)
                if type(value) in (int, float) and value != base[group].get(key):
                    candidate[group][key] = round_design_input(group, key, value)
    if isinstance(candidate.get("name"), str):
        candidate["name"] = candidate["name"].strip()
    return candidate


def retained_value_label(field):
    path = field.path
    if path[0] == "hardpoints" and len(path) == 5:
        return f"{path[2]} · {POINT_LABELS[path[3]]} · {'XYZ'[path[4]]}"
    if path[0] == "arm_mount_x":
        return f"{GROUP_NAMES[path[1]]} · {MOUNT_LABELS[path[2]]}"
    if path[0] == "svsa_design":
        return f"{GROUP_NAMES[path[1]]} · Set SVSA directly"
    label = FIELD_LABELS[path[-1]]
    return f"{GROUP_NAMES[path[0]]} · {label}" if path[0] in GROUP_NAMES else label


def import_response(project, missing):
    if not missing:
        return {**project_response(validated_engine(project)), "missing": []}
    fields = []
    for field in missing:
        default = "Baseline Vehicle" if field.path == ("name",) else field.default
        displayed = default * field.scale if type(default) in (int, float) else default
        fields.append({"path": field.path, "location": field.location, "label": retained_value_label(field),
                       "unit": unit_label(field.unit), "default": default, "displayDefault": displayed, "scale": field.scale})
    return {"project": project, "missing": fields}


def import_project(text):
    with TemporaryDirectory() as temporary:
        path = Path(temporary) / "project.json"
        path.write_text(text, encoding="utf-8")
        try:
            project, missing = read_import_project(path)
        except json.JSONDecodeError as error:
            raise ValueError(f"Invalid JSON at line {error.lineno}, column {error.colno}.") from error
    return import_response(project, missing)


def repair_project(payload):
    fields, values = [], []
    for field, value in zip(payload["missing"], payload["values"], strict=True):
        default = field["default"]
        if type(default) in (int, float) or default is None and value.strip():
            if value.strip():
                try:
                    valid = math.isfinite(float(value))
                except ValueError:
                    valid = False
                if not valid:
                    raise ValueError(f"Enter a finite number for {field['label']}.")
            if default is None:
                default = 0.0
        fields.append(MissingField(tuple(field["path"]), field["label"], field["unit"], default, field["scale"]))
        if isinstance(default, bool):
            value = "是" if value == "true" else "否" if value == "false" else ""
        values.append(value)
    return import_response(fill_import_project(payload["project"], fields, values), [])


def solve_pose(engine, payload):
    page = payload["page"]
    mode = payload.get("mode", "travel")
    x, y = payload.get("values", (0.0, 0.0))
    if page == "sv":
        return engine.sv_pose(x, y) if mode == "pose" else engine.solve_sv(x, y)
    if page == "we":
        return engine.solve_we(x, payload.get("axle", "front"))
    if page in ("front", "rear"):
        motion = engine.fv[page]
        return motion.pose(x, y) if mode == "pose" else motion.solve(x, y)
    return {}


def csv_download(rows):
    keys = list(dict.fromkeys(key for row in rows for key in row))
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=keys)
    writer.writeheader()
    for row in rows:
        if row.get("error"):
            row = {**row, "error": english_error(row["error"])}
        writer.writerow(row)
    return {"content": "\ufeff" + stream.getvalue(), "mime": "text/csv;charset=utf-8", "extension": "csv"}


def json_value(value):
    if hasattr(value, "tolist"):
        return value.tolist()
    raise TypeError(f"Cannot serialize {type(value).__name__}.")


def report_download(engine, payload, state, mapping):
    page = payload["page"]
    metrics = (state or {}).get("metrics", {})
    controls = {key: payload[key] for key in ("page", "plot", "mode", "axle", "values", "groups", "staticReferences", "construction", "lapName", "showLap", "lapVisibleSeries", "lapNormalCurves") if key in payload}
    snapshot = json.dumps({"project": engine.project, "view_controls": controls,
                           "static_parameters": engine.derived_parameters, "mount_position_checks": mount_checks(engine),
                           "contact_frame": (state or {}).get("frame")}, ensure_ascii=False, indent=2, default=json_value)
    if not metrics and page == "static":
        metrics = {"Front hardpoints": 12, "Rear hardpoints": 12}
    metric_table = "".join(
        f"<tr><th scope='row'>{html.escape(name.replace('_', ' '))}</th>"
        f"<td>{html.escape('N/A' if value is None else format(value, '.7g') if isinstance(value, (int, float)) else str(value))}</td></tr>"
        for name, value in metrics.items()
    )
    coverage = payload.get("coverage") or mapping or {}
    coverage_text = (f"The scan reached {coverage['valid']:,} of {coverage['total']:,} samples."
                     if "valid" in coverage and "total" in coverage else "")
    if "straight" in coverage:
        coverage_text += f" Excluded straight-line samples: {coverage['straight']:,}."
    svg = payload.get("svg", "")
    encoded_chart = base64.b64encode(svg.encode()).decode() if svg else ""
    chart = f'<img alt="Suspension geometry chart" src="data:image/svg+xml;base64,{encoded_chart}">' if svg else ""
    title = html.escape(PAGE_NAMES[page])
    method = "Hardpoint geometry from 24 vehicle coordinates." if hardpoints.enabled(engine.project) else "Static geometry solved from front and rear axle design parameters."
    content = f'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>SuspensionGeometryLab · {title}</title><style>
body{{font:15px "Segoe UI",sans-serif;margin:32px auto;padding:0 20px;max-width:1040px;color:#15223a}}
table{{border-collapse:collapse}}th,td{{border:1px solid #d9e2ec;padding:6px 12px;text-align:left}}
img{{display:block;max-width:100%;height:auto}}pre{{white-space:pre-wrap;overflow-wrap:anywhere;background:#eef2f7;padding:16px}}
button{{padding:8px 14px}}@media print{{button{{display:none}}img,tr{{break-inside:avoid}}}}
</style></head><body><button onclick="window.print()">Print / Save as PDF</button>
<h1>SuspensionGeometryLab · {title}</h1>
<p>{html.escape(engine.project['name'])} · {datetime.now(timezone(timedelta(hours=8))).isoformat(timespec='seconds')}</p>
<p>{method} {REPORT_NOTES[page]}</p><p>{coverage_text}</p>{chart}
<h2>Current pose</h2><table>{metric_table}</table>
<h2>Project and view settings</h2><pre>{html.escape(snapshot)}</pre></body></html>'''
    return {"content": content, "mime": "text/html;charset=utf-8", "extension": "html"}


def export_data(payload, engine, state, mapping):
    page, kind = payload["page"], payload["kind"]
    if engine is None:
        engine = Engine(payload["project"])
    if kind == "mapping":
        if page == "static":
            raise ValueError("Static Vehicle provides a Hardpoints CSV. Use an analysis view for a sweep CSV.")
        if mapping is None:
            if payload.get("plot") == "lap_camber":
                raise ValueError("Load lap data and calculate the camber distribution before exporting.")
            mapping = calculate_mapping(engine, page, payload.get("mode", "travel"), payload.get("axle", "front"))
        return csv_download(mapping["rows"])
    if state is None and page != "static" and kind != "report":
        state = solve_pose(engine, payload)
    if kind in ("state", "points"):
        rows = static_point_rows(engine) if page == "static" else point_rows(state) if kind == "points" else [state["metrics"]]
        return csv_download(rows)
    if kind == "report":
        return report_download(engine, payload, state, mapping)
    if kind == "step":
        if page not in STEP_PAGES:
            raise ValueError(EXACT_ERRORS["STEP 匯出適用於前 FV、後 FV、SV 與靜態全車。"])
        options = {"show_construction": payload.get("construction", True), **payload.get("stepOptions", {})}
        with TemporaryDirectory() as temporary:
            destination = Path(temporary) / "suspension.step"
            write_step(destination, engine, page, state, **options)
            return {"content": destination.read_text(encoding="ascii"), "mime": "model/step", "extension": "step"}
    raise ValueError(f"Unknown download format {kind}.")


def handle(action, payload, *, engine=None, state=None, mapping=None):
    try:
        if action == "project-default":
            return default_browser_project()
        if action == "project-prepare":
            return prepare_project(payload["project"])
        if action == "project-apply":
            return project_response(validated_engine(editable_project(payload)))
        if action == "project-preview":
            project = editable_project(payload)
            if "hardpoints" in project:
                project["hardpoints"]["enabled"] = payload["mode"] == "hardpoint"
            candidate = Engine(project)
            return {**project_response(candidate), "points": hardpoints.from_scene(candidate.scene)}
        if action == "project-import":
            return import_project(payload["text"])
        if action == "project-repair":
            return repair_project(payload)
        if action == "export-data":
            return export_data(payload, engine, state, mapping)
        raise ValueError(f"Unknown project action {action}.")
    except (ArithmeticError, ValueError) as error:
        message = english_error(error)
        if message == str(error):
            raise
        raise ValueError(message) from error
