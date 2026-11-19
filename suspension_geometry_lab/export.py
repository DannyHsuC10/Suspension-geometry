"""Export project results as CSV, HTML reports and STEP wireframes or solids."""
import csv
import datetime
import html
import json
import math
from pathlib import Path
from tempfile import NamedTemporaryFile

import numpy as np

from .analysis import RESULT_LABELS, NOTES, PAGES
from .engine import to_contact_frame
from .static.fv.wireframe_geometry import project_fv
from .step_solids import write_cylinders
from .wheel_alignment import installation_matrix
from .wheel_end_geometry import rotation_matrix


STEP_PAGES = {"front": "Front FV", "rear": "Rear FV", "sv": "Side view", "static": "Full vehicle links"}


def write_csv(path, rows):
    keys = list(dict.fromkeys(k for row in rows for k in row))
    with Path(path).open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def point_rows(state):
    rows = []
    for name, p in state["points"].items():
        row = {"name": name, **{axis+"_m": value for axis, value in zip("xyz" if len(p) == 3 else "xz", p)}}
        if "frame" in state and len(p) == 2:
            local = to_contact_frame(p, state["frame"])
            row.update(x_prime_m=float(local[0]), y_prime_m=float(local[1]))
        rows.append(row)
    return rows


def _step_polylines(engine, page, state, show_construction):
    if page == "static":
        geometry = engine.scene.geometry
        for corner, joints in geometry["kingpins"].items():
            axle = "rear_" if corner.startswith("R") else ""
            for arm, joint in (("upper", "UBJ"), ("lower", "LBJ")):
                mounts = geometry[f"{axle}{arm}_arm_mount_geometry"][corner]["mounts"]
                for name, endpoint in mounts.items():
                    yield f"{corner} {arm} arm {name}", (joints[joint], endpoint)
            yield f"{corner} upright", (joints["LBJ"], joints["UBJ"])
        for corner, endpoints in engine.static_tie_rods():
            yield f"{corner} tie rod", endpoints
        return

    if not state:
        raise ValueError("目前姿態無解；請調整滑桿後再匯出。")
    for path in state["lines"]:
        # Replace the FV rectangle / SV center-plane circle with the real
        # two-circle 3D tire below, or omit it when tires are unchecked.
        if path["name"] in ("wheel", "Front wheel", "Rear wheel"):
            continue
        # The contact-frame arrows do not participate in geometry fitting.
        if not path["fit"] or (path["construction"] and not show_construction):
            continue
        if page == "sv":
            points = engine.scene.geometry["side_view"].project(path["points"])
        else:
            points = project_fv(path["points"], engine.scene.geometry[page].longitudinal_x)
        yield path["name"], points


def _step_wheels(engine, page, state, include_static_camber):
    """Yield wheel centers and orientations in fixed vehicle coordinates.

    Only the tire installation loses its static camber when unchecked. Current
    joint motion, wheel-center positions and the exported mechanism stay fixed.
    """
    def installation(corner):
        axle = "front" if corner.startswith("F") else "rear"
        gamma = engine.project[axle]["static_camber_deg"] if include_static_camber else 0.
        return installation_matrix(gamma, corner)

    if page == "static":
        for corner, wheel in engine.scene.geometry["wheel_projection_geometry"].items():
            yield corner, np.asarray(wheel["center"]), installation(corner)
        return
    if not state:
        raise ValueError("目前姿態無解；請調整滑桿後再匯出。")
    points = state["points"]
    if page in ("front", "rear"):
        prefix = "F" if page == "front" else "R"
        zero = engine.fv[page].zero
        reference = zero["kua"]-zero["lak"]
        axle_x = engine.scene.geometry[page].longitudinal_x
        for suffix in ("L", "R"):
            corner = prefix+suffix
            current = np.asarray(points[suffix+"_kua"])-points[suffix+"_lak"]
            initial = reference*([-1., 1.] if suffix == "L" else [1., 1.])
            angle = math.atan2(current[1], current[0])-math.atan2(initial[1], initial[0])
            center = project_fv(points[suffix+"_wheel_center"], axle_x)
            yield corner, center, rotation_matrix([1., 0., 0.], -angle)@installation(corner)
        return
    # The SV mechanism remains at Y=0. Mirror the axle-average wheel pose at
    # the actual track positions to give four tires in the exported wireframe.
    sv = engine.scene.geometry["side_view"]
    for axle, prefix, track in (("front", "F", engine.car.tf), ("rear", "R", engine.car.tr)):
        template = engine.sv[axle].template
        initial = template.kingpin_Au-template.kingpin_Al
        current = np.asarray(points[axle+"_au"])-points[axle+"_al"]
        angle = math.atan2(current[1], current[0])-math.atan2(initial[1], initial[0])
        center = sv.project(points[axle+"_center"])
        motion = rotation_matrix([0., 1., 0.], angle)
        for suffix, side in (("L", 1), ("R", -1)):
            corner = prefix+suffix
            position = center.copy()
            position[1] = side*track/2
            yield corner, position, motion@installation(corner)


def _step_cylinders(engine, page, state, include_tires, include_static_camber, tube_diameter_mm):
    diameter = float(tube_diameter_mm)
    if not math.isfinite(diameter) or diameter <= 0:
        raise ValueError("懸吊管徑必須是大於 0 的有限數值（mm）。")
    for name, points in _step_polylines(engine, page, state, False):
        if page != "static" and name not in ("upper", "lower", "upright"):
            continue
        for start, end in zip(points[:-1], points[1:]):
            yield name, start, end, diameter/2000
    if include_tires:
        radius, width = float(engine.car.free_radius), float(engine.car.tire_width)
        if not math.isfinite(radius) or not math.isfinite(width) or radius <= 0 or width <= 0:
            raise ValueError("STEP 輪胎半徑與胎寬必須是正的有限數值。")
        for corner, center, matrix in _step_wheels(engine, page, state, include_static_camber):
            offset = width/2*matrix[:, 1]
            yield corner+" tire", center-offset, center+offset, radius


def write_step(path, engine, page, state=None, *, show_construction=True,
               include_tires=True, include_static_camber=True,
               export_solids=False, tube_diameter_mm=20.):
    """Write an AP214 wireframe or independent solid cylinders, in mm."""
    if page not in STEP_PAGES:
        raise ValueError("STEP 匯出適用於前 FV、後 FV、SV 與靜態全車。")
    if export_solids:
        cylinders = _step_cylinders(engine, page, state, include_tires, include_static_camber, tube_diameter_mm)
        description = "static camber enabled" if include_tires and include_static_camber else "static camber disabled"
        write_cylinders(path, cylinders, description)
        return
    entities = []

    def entity(value):
        entities.append(value)
        return f"#{len(entities)}"

    application = entity("APPLICATION_CONTEXT('automotive_design')")
    entity(f"APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,{application})")
    context = entity(f"PRODUCT_CONTEXT('',{application},'mechanical')")
    product = entity(f"PRODUCT('SuspensionGeometryLab','{STEP_PAGES[page]}','',({context}))")
    formation = entity(f"PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE('','',{product},.NOT_KNOWN.)")
    definition_context = entity(f"PRODUCT_DEFINITION_CONTEXT('part definition',{application},'design')")
    definition = entity(f"PRODUCT_DEFINITION('design','',{formation},{definition_context})")
    shape = entity(f"PRODUCT_DEFINITION_SHAPE('','',{definition})")
    mm = entity("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))")
    angle = entity("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))")
    solid_angle = entity("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())")
    uncertainty = entity(f"UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.000001),{mm},'distance_accuracy_value','')")
    representation_context = entity(
        f"(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT(({uncertainty}))"
        f"GLOBAL_UNIT_ASSIGNED_CONTEXT(({mm},{angle},{solid_angle}))REPRESENTATION_CONTEXT('','3D'))")

    curves, point_refs = [], {}

    def coordinates(values, scale=1.):
        xyz = tuple(float(value)*scale for value in values)
        if len(xyz) != 3 or not all(map(math.isfinite, xyz)):
            raise ValueError("STEP 座標與方向必須是三個有限數值。")
        return ",".join(f"{round(value, 9)+0.0:.9f}" for value in xyz)

    def cartesian_point(values):
        coords = coordinates(values, 1000.)
        if coords not in point_refs:
            point_refs[coords] = entity(f"CARTESIAN_POINT('',({coords}))")
        return point_refs[coords]

    for name, points in _step_polylines(engine, page, state, show_construction):
        vertices = []
        for point in points:
            xyz = tuple(float(value) * 1000 for value in point)
            if not all(map(math.isfinite, xyz)):
                raise ValueError(f"{name}：STEP 座標必須為有限數值。")
            coords = ",".join(f"{round(value, 9) + 0.0:.9f}" for value in xyz)
            if not vertices or vertices[-1] != coords:
                vertices.append(coords)
        if len(vertices) < 2:
            continue
        for coords in vertices:
            if coords not in point_refs:
                point_refs[coords] = entity(f"CARTESIAN_POINT('',({coords}))")
        refs = ",".join(point_refs[coords] for coords in vertices)
        curves.append(entity(f"POLYLINE('{name}',({refs}))"))
    if include_tires:
        radius, width = float(engine.car.free_radius), float(engine.car.tire_width)
        if not math.isfinite(radius) or not math.isfinite(width) or radius <= 0 or width <= 0:
            raise ValueError("STEP 輪胎半徑與胎寬必須是正的有限數值。")
        for corner, center, matrix in _step_wheels(engine, page, state, include_static_camber):
            normal, reference = matrix[:, 1], matrix[:, 0]
            axis = entity(f"DIRECTION('',({coordinates(normal)}))")
            ref = entity(f"DIRECTION('',({coordinates(reference)}))")
            for suffix, side in (("-", -1), ("+", 1)):
                location = cartesian_point(center+side*width/2*normal)
                placement = entity(f"AXIS2_PLACEMENT_3D('',{location},{axis},{ref})")
                curves.append(entity(f"CIRCLE('{corner} tire side {suffix}',{placement},{radius*1000:.9f})"))
    if not curves:
        raise ValueError("目前沒有可匯出的 STEP 線框。")
    curve_set = entity(f"GEOMETRIC_CURVE_SET('',({','.join(curves)}))")
    representation = entity(
        f"GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION('',({curve_set}),{representation_context})")
    entity(f"SHAPE_DEFINITION_REPRESENTATION({shape},{representation})")

    tire_description = "two circles per tire" if include_tires else "links only"
    camber_description = "static camber enabled" if include_static_camber and include_tires else "static camber disabled"
    header = ("ISO-10303-21;\nHEADER;\n"
              f"FILE_DESCRIPTION(('SuspensionGeometryLab wireframe','{tire_description}','{camber_description}'),'2;1');\n"
              "FILE_NAME('suspension.step','',(''),(''),'SuspensionGeometryLab','SuspensionGeometryLab','');\n"
              "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n")
    data = "\n".join(f"#{i}={value};" for i, value in enumerate(entities, start=1))
    content = (header + data + "\nENDSEC;\nEND-ISO-10303-21;\n").encode("ascii")
    destination = Path(path)
    temporary = None
    try:
        with NamedTemporaryFile(dir=destination.parent, prefix=".step-", delete=False) as handle:
            temporary = Path(handle.name)
            handle.write(content)
        temporary.replace(destination)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def write_report(path, project, page, state, mapping, svg, controls, *, derived_parameters=None, mount_position_checks=None):
    esc = lambda value: html.escape(str(value))
    metrics = state.get("metrics", {}) if state else {}
    metric_table = "".join(
        f"<tr><th>{esc(RESULT_LABELS.get(k, k))}</th>"
        f"<td>{esc('N/A' if v is None else format(v, '.7g') if isinstance(v, (float, int)) else v)}</td></tr>"
        for k, v in metrics.items())
    coverage = f"範圍掃描：{mapping['valid']} / {mapping['total']} 筆有效。缺值代表不可達姿態或無有限交點。" if mapping else "本報告未包含範圍掃描。"
    if mapping and "straight" in mapping:
        coverage = (f"圈速檔案 {esc(controls.get('lap_csv', ''))} 有 {mapping['valid']} / {mapping['total']} 筆"
                    f"有效彎道資料，排除 {mapping['straight']} 筆直線樣本。")
    snapshot = json.dumps({"project": project, "view_controls": controls,
                           "static_parameters": derived_parameters,
                           "mount_position_checks": mount_position_checks,
                           "contact_frame": (state or {}).get("frame")}, ensure_ascii=False, indent=2)
    method = ("硬點模式：以 24 個 XYZ 定義幾何；FV 使用等效樞點，SV 使用硬點軸線的固定切線近似，WE 使用 UBJ–LBJ 主銷。"
              "快照內 front／rear 是保留的參數模式設計輸入，static_parameters 才是目前硬點反算值。"
              if project.get("hardpoints", {}).get("enabled") else "參數模式：由前後軸設計參數解算靜態幾何。")
    content = f'''<!doctype html><html lang="zh-Hant"><meta charset="utf-8">
<title>SuspensionGeometryLab · {esc(PAGES[page])}</title><style>
body{{font:14px 'Microsoft JhengHei',sans-serif;margin:32px;color:#172b4d}}h1{{font-size:26px}}
table{{border-collapse:collapse}}th,td{{padding:5px 12px;border:1px solid #ccd5df;text-align:left}}
svg{{max-width:100%;height:auto}}pre{{white-space:pre-wrap;background:#f1f5f9;padding:16px}}
@media print{{button{{display:none}}svg,table{{break-inside:avoid}}}}
</style><button onclick="window.print()">列印 / 另存 PDF</button>
<h1>SuspensionGeometryLab · {esc(PAGES[page])}</h1><p>{esc(project['name'])} · {datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(timespec='seconds')}</p>
<p>{esc(method)}</p><p>{esc(NOTES[page])}</p><p>{esc(coverage)}</p>{svg}<h2>目前姿態</h2><table>{metric_table}</table>
<h2>可重現輸入快照</h2><pre>{esc(snapshot)}</pre>
<p>完整範圍資料與命名硬點請使用介面底部的 CSV 匯出。曲面座標依圖示單位，硬點 CSV 為 m。</p></html>'''
    Path(path).write_text(content, encoding="utf-8")
