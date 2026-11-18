"""Lengthen FVSA until the first lap camber limit or geometry failure."""
import copy
from dataclasses import dataclass
from decimal import Decimal, ROUND_FLOOR
import math

import numpy as np

from .engine import Engine, Unreachable
from .hardpoints import enabled
from .input_limits import AXLE_LIMITS, INPUT_STEPS
from .project import validate
from .roll_center_search import SearchCancelled


@dataclass(frozen=True)
class CamberPeak:
    peak_deg: float
    signed_deg: float
    sample: int
    wheel: str
    total: int


@dataclass(frozen=True)
class FVSASearchResult:
    project: dict | None
    peak: CamberPeak | None
    tested: int
    total: int
    stop_reason: str
    stopped_fvsa_m: float | None = None
    stopped_peak: CamberPeak | None = None
    error: str = ""


def prepare_lap(lap):
    if not lap.travels or len(lap.travels) != len(lap.poses):
        raise ValueError("FVSA 搜尋需要四輪相對位移、heave_mm、roll_deg、pitch_deg，"
                         "以及 curvature_1_m 或 ay_g 欄位；CSV 的每筆資料均須有車身姿態。")
    travels = np.asarray(lap.travels, dtype=float)
    poses = np.asarray([(p.heave_mm, p.roll_deg, p.pitch_deg, p.turn_signal) for p in lap.poses])
    if travels.shape != (len(lap.poses), 4) or not np.all(np.isfinite(travels)) or not np.all(np.isfinite(poses)):
        raise ValueError("CSV 的四輪行程與車身姿態必須完整且為有限數值。")
    return travels, np.deg2rad(poses[:, 1]), np.deg2rad(poses[:, 2])


def fvsa_candidates(project, axle):
    # Start at the exact half-track, including half increments for odd track
    # widths; subsequent candidates differ by the existing FVSA edit increment.
    start = Decimal(str(project["vehicle"]["track_"+axle]))/2
    step = Decimal(str(INPUT_STEPS["fvsa"]))
    count = int(((Decimal(str(AXLE_LIMITS["fvsa"].high))-start)/step).to_integral_value(rounding=ROUND_FLOOR))
    return [float(start+i*step) for i in range(count+1)]


def _at_angles(motion, theta):
    """Vector form of FVMotion.at_angle's same closest-to-zero assembly branch."""
    z = motion.zero
    lower = z["cpb_rl"]+motion.lower*np.column_stack((np.cos(theta), np.sin(theta)))
    delta = lower-z["cpb_ru"]
    distance = np.linalg.norm(delta, axis=1)
    valid = ((distance >= 1e-12) & (distance <= motion.upper+motion.upright)
             & (distance >= abs(motion.upper-motion.upright)))
    distance = np.where(valid, distance, np.nan)
    a = (motion.upper**2-motion.upright**2+distance**2)/(2*distance)
    h = np.sqrt(np.maximum(0., motion.upper**2-a**2))
    point = z["cpb_ru"]+a[:, None]*delta/distance[:, None]
    normal = np.column_stack((-delta[:, 1], delta[:, 0]))/distance[:, None]
    first, second = point+h[:, None]*normal, point-h[:, None]*normal
    use_first = np.sum((first-z["kua"])**2, axis=1) <= np.sum((second-z["kua"])**2, axis=1)
    upper = np.where(use_first[:, None], first, second)
    new, old = upper-lower, z["kua"]-z["lak"]
    angle = np.arctan2(new[:, 1], new[:, 0])-math.atan2(old[1], old[0])
    sine, cosine = np.sin(angle), np.cos(angle)
    heights = []
    for key in ("wheel_center", "gnd"):
        offset = z[key]-z["lak"]
        heights.append(lower[:, 1]+sine*offset[0]+cosine*offset[1])
    return heights[0], heights[1], angle


def _wheel_center_motion(motion, travels, cancelled):
    """Batch the existing bracket/refinement solve, retaining scalar fallback.

    The 81-point branch grid, closest bracket, residual and 90-step refinement
    agree with scalar_solve. Only angle/contact height are needed for camber.
    """
    target = motion.zero["wheel_center"][1]+travels/1000
    samples = motion.motion_samples
    coordinates = np.asarray([x for x, _ in samples])
    values = np.asarray([s["wheel_center"][1] if "wheel_center" in s else np.nan for _, s in samples])
    residual = values[None, :]-target[:, None]
    brackets = (np.isfinite(residual[:, :-1]) & np.isfinite(residual[:, 1:])
                & (residual[:, :-1]*residual[:, 1:] <= 0))
    order = np.argsort(np.abs((coordinates[:-1]+coordinates[1:])/2-motion.theta0), kind="stable")
    indices = order[np.argmax(brackets[:, order], axis=1)]
    rows = np.arange(len(target))
    lo, hi = coordinates[indices].copy(), coordinates[indices+1].copy()
    flo, fhi = residual[rows, indices].copy(), residual[rows, indices+1].copy()
    roots = np.full(len(target), np.nan)
    at_zero = np.abs(target-motion.zero["wheel_center"][1]) < 1e-11
    roots[at_zero] = motion.theta0
    active = np.any(brackets, axis=1) & ~at_zero
    for bound, errors in ((lo, flo), (hi, fhi)):
        exact = active & (np.abs(errors) < 1e-11)
        roots[exact] = bound[exact]
        active[exact] = False
    for iteration in range(90):
        if cancelled():
            raise SearchCancelled()
        pending = np.flatnonzero(active)
        if not len(pending):
            break
        mid = ((lo[pending]+hi[pending])/2 if iteration % 2 else
               lo[pending]-flo[pending]*(hi[pending]-lo[pending])/(fhi[pending]-flo[pending]))
        fm = _at_angles(motion, mid)[0]-target[pending]
        converged = np.isfinite(fm) & (np.abs(fm) < 1e-11)
        roots[pending[converged]] = mid[converged]
        active[pending[converged | ~np.isfinite(fm)]] = False
        update = ~converged & np.isfinite(fm)
        upper = update & (flo[pending]*fm <= 0)
        lower = update & ~upper
        hi[pending[upper]], fhi[pending[upper]] = mid[upper], fm[upper]
        lo[pending[lower]], flo[pending[lower]] = mid[lower], fm[lower]
    _, ground, angles = _at_angles(motion, roots)
    # A rare failed bracket/refinement uses the authoritative scalar solver,
    # including its next-nearest branch search and Unreachable reporting.
    for index in np.flatnonzero(~np.isfinite(roots)):
        if cancelled():
            raise SearchCancelled()
        state = motion.side(float(travels[index]), wheel_center=True)
        ground[index], angles[index] = state["gnd"][1], state["angle"]
    contact_travels = (ground-motion.zero["gnd"][1])*1000
    low, high = motion.travel_bounds
    invalid = (~np.isfinite(angles) | ~np.isfinite(contact_travels)
               | (contact_travels < low-1e-5) | (contact_travels > high+1e-5))
    if np.any(invalid):
        index = np.flatnonzero(invalid)[0]
        raise Unreachable(f"輪心行程 {travels[index]:g} mm 對應的接地行程超出設定或無解。")
    return angles


def _lap_peak(engine, axle, prepared, absolute, cancelled):
    travels, roll, pitch = prepared
    offset = 0 if axle == "front" else 2
    pair = travels[:, offset:offset+2]
    unique, inverse = np.unique(pair, return_inverse=True)
    angle = _wheel_center_motion(engine.fv[axle], unique, cancelled)[inverse].reshape(pair.shape)
    angle = angle-math.radians(engine.project[axle]["static_camber_deg"])
    # Same body-to-road convention as calculate_lap_camber, including straight
    # samples. Left states mirror the right-angle sign; outward axes mirror Y.
    road_y, road_z = np.cos(pitch)*np.sin(roll), np.cos(pitch)*np.cos(roll)
    vertical = road_y[:, None]*np.cos(angle)*[1., -1.]+road_z[:, None]*np.sin(angle)
    camber = -np.rad2deg(np.arcsin(np.clip(vertical, -1., 1.)))
    if not np.all(np.isfinite(camber)):
        raise ValueError("賽道姿態產生無效的 Camber。")
    metric = np.abs(camber) if absolute else camber
    sample, wheel = np.unravel_index(np.argmax(metric), metric.shape)
    prefix = "F" if axle == "front" else "R"
    return CamberPeak(float(metric[sample, wheel]), float(camber[sample, wheel]),
                      int(sample)+1, prefix+("L" if wheel == 0 else "R"), len(pair))


def calculate_lap_peak(engine, axle, lap, *, absolute=True, cancelled=lambda: False):
    return _lap_peak(engine, axle, prepare_lap(lap), absolute, cancelled)


def validate_camber_limit(value):
    if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= 90:
        raise ValueError("最大允許 Camber 須為 0～90 度的有限數值。")
    return value


def search_fvsa(project, axle, lap, max_camber_deg, *, absolute=True, cancelled=lambda: False,
                progress=lambda tested, total, fvsa, peak, reason: None):
    project = validate(project)
    if axle not in ("front", "rear"):
        raise ValueError("FVSA 搜尋只適用於前／後軸 FV。")
    if enabled(project):
        raise ValueError("硬點模式的 FVSA 為反算結果；請切換至參數模式後搜尋。")
    validate_camber_limit(max_camber_deg)
    prepared = prepare_lap(lap)
    lengths = fvsa_candidates(project, axle)
    candidate = copy.deepcopy(project)
    best, best_peak = None, None
    for tested, length in enumerate(lengths, 1):
        if cancelled():
            raise SearchCancelled()
        candidate[axle]["fvsa"] = length
        try:
            engine = Engine(candidate)
            if any(check["fallback"] for checks in engine.scene.geometry.get("mount_position_checks", {}).values()
                   for check in checks.values() if check["requested_x"] is not None):
                raise ValueError("A 臂車身 X 超出候選幾何的有效範圍。")
            peak = _lap_peak(engine, axle, prepared, absolute, cancelled)
        except (ValueError, ArithmeticError) as exc:
            if cancelled():
                raise SearchCancelled()
            progress(tested, len(lengths), length, None, str(exc))
            return FVSASearchResult(best, best_peak, tested, len(lengths), "geometry", length, error=str(exc))
        if cancelled():
            raise SearchCancelled()
        progress(tested, len(lengths), length, peak, "")
        if peak.peak_deg > max_camber_deg:
            return FVSASearchResult(best, best_peak, tested, len(lengths), "camber_limit", length, peak)
        best, best_peak = copy.deepcopy(engine.project), peak
    return FVSASearchResult(best, best_peak, len(lengths), len(lengths), "upper_limit")
