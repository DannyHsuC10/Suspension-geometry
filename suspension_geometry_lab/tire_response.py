"""Pacejka contact loads and their moments about the suspension kingpin."""

import math
from pathlib import Path

import numpy as np

from ._pacejka import tir_io
from ._pacejka.model import api


TIR_DIR = Path(tir_io.__file__).with_name("tir")
DEFAULT_TIR = "Hoosier_43075_R20_W8_direct_candidate.tir"
CORNERS = ("FL", "FR", "RL", "RR")
FRONT_CORNERS = ("FL", "FR")
UP = np.array([0., 0., 1.])
AXLE_SUM_FIELDS = ("mz_nm", "trail_moment_nm", "aligning_total_nm",
                   "torque_total_nm", "torque_mz_nm", "torque_contact_nm",
                   "torque_fx_nm", "torque_fy_nm", "torque_fz_nm", "fz_n")
FIT_RANGES = (
    ("輪荷", "FZMIN", "FZMAX"),
    ("胎壓", "PRESMIN", "PRESMAX"),
    ("滑移率", "KPUMIN", "KPUMAX"),
    ("側偏角", "ALPMIN", "ALPMAX"),
    ("外傾角", "CAMMIN", "CAMMAX"),
)


def load_tire_parameters(path):
    params = tir_io.load_tir(path)
    side = str(params.get("TYRESIDE", "left")).strip().lower()
    if side not in ("left", "right", "symmetric"):
        raise ValueError(f"不支援的 TYRESIDE：{side}")
    for _, lower, upper in FIT_RANGES:
        for key in (lower, upper):
            if key in params and not isinstance(params[key], (int, float)):
                raise ValueError(f"TIR 的 {key} 必須是數值。")
    try:
        # A valid TIR header alone does not promise the coefficients used by Mz.
        _pacejka_loads(params, params["FNOMIN"], np.array([-1., 0., 1.]), 0., 0.,
                      params["NOMPRES"], params["LONGVL"])
    except (KeyError, AttributeError, TypeError) as exc:
        raise ValueError(f"TIR 缺少必要欄位或係數格式有誤：{exc}") from exc
    return params


def _pacejka_loads(params, fz, slip_angles, slip_ratio, ia_deg, pressure_pa, speed_mps):
    if fz == 0:
        return np.zeros((3, len(slip_angles)))
    condition = api.CalculationInput(FZ=fz, SA=slip_angles, SL=slip_ratio, IA=ia_deg,
                                     pressure=pressure_pa, V=speed_mps, tir_params=params)
    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        fx, fy = api.calculate_fx(condition), api.calculate_fy(condition)
        mz = api.calculate_mz(condition.with_inputs(Fx=fx, Fy=fy))
        loads = np.stack((fx, fy, mz))
    if not np.isfinite(loads).all():
        raise ValueError("Pacejka 在此工況無法算出有限的 Fx、Fy、Mz。")
    return loads


def tire_orientation(engine, corner, steer_deg, *, wheel_camber_deg=None, camber_offset_deg=0.):
    """Return steered geometry, wheel axes, common-frame IA and wheel camber."""
    if not math.isfinite(steer_deg):
        raise ValueError("主銷轉向角必須為有限數值。")
    steer_limit = engine.project["analysis"]["steer_deg"]
    if abs(steer_deg) > steer_limit:
        raise ValueError(f"主銷轉向角須在專案的 ±{steer_limit:g}° 範圍內。")
    if wheel_camber_deg is not None and not -90 < wheel_camber_deg < 90:
        raise ValueError("Camber 外傾角必須介於 −90° 與 90° 之間。")

    pose = engine.wheel_end_pose(steer_deg, corner)
    forward = np.cross(pose["normal"], UP)
    forward /= np.linalg.norm(forward)
    left = np.cross(UP, forward)
    # Positive IA tilts the wheel top rightward, which is inward on the left side.
    camber_to_ia = -1 if corner.endswith("L") else 1
    if wheel_camber_deg is None:
        ia = math.degrees(math.asin(float(np.clip(pose["normal"][2], -1., 1.)))) + camber_offset_deg
        if not -90 < ia < 90:
            raise ValueError("合成外傾角必須介於 −90° 與 90° 之間。")
        wheel_camber_deg = camber_to_ia * ia
    else:
        ia = camber_to_ia * wheel_camber_deg
    return pose, forward, left, ia, wheel_camber_deg


def calculate_tire_response(engine, params, *, corner, steer_deg, slip_angles_deg,
                            slip_ratio, camber_offset_deg=0., fz_n=None,
                            pressure_kpa=None, speed_mps=None, wheel_camber_deg=None):
    """Return force and torque rows for a slip-angle sweep at one steered pose.

    The wheel's contact frame uses forward, left and up axes. Positive moments
    follow the right-hand rule about the up axis (Mz) or LBJ → UBJ (kingpin).
    An explicit wheel camber (positive outward) overrides the geometry-derived IA.
    The camber_deg output retains the common-frame IA used in existing exports.
    """
    if corner not in CORNERS:
        raise ValueError("請選擇 FL、FR、RL 或 RR。")
    axle_index = 0 if corner.startswith("F") else 1
    fz = engine.car.w * engine.car.cg_rate[axle_index] / 2 if fz_n is None else fz_n
    pressure_pa = params["NOMPRES"] if pressure_kpa is None else pressure_kpa * 1000
    speed = params["LONGVL"] if speed_mps is None else speed_mps
    angles = np.atleast_1d(np.asarray(slip_angles_deg, dtype=float))
    if not np.isfinite([steer_deg, slip_ratio, camber_offset_deg, fz, pressure_pa, speed]).all():
        raise ValueError("輪胎工況必須為有限數值。")
    if angles.ndim != 1 or not angles.size or not np.all(np.abs(angles) < 90):
        raise ValueError("側偏角必須介於 −90° 與 90° 之間。")
    if fz < 0 or pressure_pa <= 0 or speed <= 0:
        raise ValueError("輪荷須大於等於零，胎壓與前進速度須大於零。")

    pose, forward, left, ia, wheel_camber_deg = tire_orientation(
        engine, corner, steer_deg, wheel_camber_deg=wheel_camber_deg, camber_offset_deg=camber_offset_deg)
    kingpin = pose["axis"]
    arm = pose["contact"] - pose["origin"]
    # About the road normal, a contact patch behind the kingpin gives -trail * Fy.
    trail = -np.dot(arm, forward)
    arm_fx, arm_fy, arm_fz = [np.dot(kingpin, np.cross(arm, direction))
                              for direction in (forward, left, UP)]

    side = str(params.get("TYRESIDE", "left")).strip().lower()
    corner_side = "left" if corner.endswith("L") else "right"
    mirror = 1 if side in ("symmetric", corner_side) else -1
    model_angles = mirror * angles
    model_camber = mirror * ia
    fx, fy, mz = _pacejka_loads(params, fz, model_angles, slip_ratio, model_camber, pressure_pa, speed)
    fy, mz = fy * mirror, mz * mirror
    trail_moment = -trail * fy
    aligning_total = mz + trail_moment
    torque_fx, torque_fy, torque_fz = arm_fx * fx, arm_fy * fy, arm_fz * fz
    torque_contact = torque_fx + torque_fy + torque_fz
    torque_mz = np.dot(kingpin, UP) * mz
    torque_total = torque_contact + torque_mz
    rows = [
        dict(corner=corner, steer_deg=steer_deg, slip_angle_deg=float(alpha), slip_ratio=slip_ratio,
             model_slip_angle_deg=float(model_angles[i]), model_camber_deg=model_camber,
             camber_deg=ia, wheel_camber_deg=wheel_camber_deg, pressure_kpa=pressure_pa / 1000, speed_mps=speed,
             fx_n=float(fx[i]), fy_n=float(fy[i]), fz_n=float(fz), mz_nm=float(mz[i]),
             trail_moment_nm=float(trail_moment[i]), aligning_total_nm=float(aligning_total[i]),
             torque_fx_nm=float(torque_fx[i]), torque_fy_nm=float(torque_fy[i]),
             torque_fz_nm=float(torque_fz), torque_contact_nm=float(torque_contact[i]),
             torque_mz_nm=float(torque_mz[i]), torque_total_nm=float(torque_total[i]))
        for i, alpha in enumerate(angles)
    ]
    outside = []
    evaluated_ranges = (
        (fz, fz), (pressure_pa, pressure_pa), (slip_ratio, slip_ratio),
        np.deg2rad([model_angles.min(), model_angles.max()]),
        (math.radians(model_camber),) * 2,
    )
    for (label, low, high), (minimum, maximum) in zip(FIT_RANGES, evaluated_ranges):
        if minimum < params.get(low, -math.inf) or maximum > params.get(high, math.inf):
            outside.append(label)
    warnings = []
    if fz == 0:
        warnings.append("輪胎離地，接地力與力矩皆為零。")
    elif outside:
        warnings.append("超出 TIR 標示範圍，曲線含外插值：" + "、".join(outside))
    if speed < params.get("VXLOW", 0):
        warnings.append("速度低於 TIR 的 VXLOW，請以穩態曲線解讀。")
    return rows, warnings


def calculate_front_axle_response(engine, params, *, left, right, slip_offsets_deg):
    """Combine two wheel responses after conversion to the vehicle sign convention."""
    offsets = np.atleast_1d(np.asarray(slip_offsets_deg, dtype=float))
    by_corner = {}
    notices = []
    for corner, condition in zip(FRONT_CORNERS, (left, right)):
        inputs = dict(condition)
        alpha = inputs.pop("slip_angle_deg")
        try:
            by_corner[corner], warnings = calculate_tire_response(
                engine, params, corner=corner, slip_angles_deg=alpha + offsets, **inputs)
        except (ValueError, ArithmeticError) as exc:
            raise ValueError(f"{corner}：{exc}") from exc
        notices.extend(f"{corner}：{warning}" for warning in warnings)

    rows = []
    for offset, fl, fr in zip(offsets, by_corner["FL"], by_corner["FR"], strict=True):
        row = {"slip_offset_deg": float(offset)}
        for prefix, wheel in (("fl", fl), ("fr", fr)):
            row.update({f"{prefix}_{key}": value for key, value in wheel.items() if key != "corner"})
        row.update({f"front_{key}": fl[key] + fr[key] for key in AXLE_SUM_FIELDS})
        rows.append(row)
    return rows, notices
