"""Fit the existing outer-wheel steered-camber distribution at 3–6° caster."""
import copy
from dataclasses import dataclass
from decimal import Decimal, ROUND_FLOOR
import math

import numpy as np

from .analysis import normal_camber_curve
from .engine import Engine
from .fvsa_search import prepare_lap
from .hardpoints import enabled
from .input_limits import INPUT_STEPS
from .lap_data import FRONT_STEER_COLUMNS, TURN_COLUMNS, calculate_lap_camber
from .project import validate
from .roll_center_search import SearchCancelled


CASTER_MIN_DEG = 3.
CASTER_MAX_DEG = 6.
OUTER_STEERED_CAMBER = "outer_camber_steered_deg"


@dataclass(frozen=True)
class CasterCamberFit:
    caster_deg: float
    mean_deg: float
    std_deg: float
    samples: int
    minimum_deg: float
    maximum_deg: float

    @property
    def error_deg(self):
        # The normal distribution's peak location is its mean, not its height.
        return abs(self.mean_deg)


@dataclass(frozen=True)
class CasterSearchResult:
    project: dict | None
    best: CasterCamberFit | None
    before: CasterCamberFit | None
    mapping: dict | None
    tested: int
    total: int
    rejected: dict


def caster_candidates():
    low, high = Decimal(str(CASTER_MIN_DEG)), Decimal(str(CASTER_MAX_DEG))
    step = Decimal(str(INPUT_STEPS["caster_deg"]))
    count = int(((high-low)/step).to_integral_value(rounding=ROUND_FLOOR))
    return [float(low+index*step) for index in range(count+1)]


def validate_caster_lap(lap):
    prepare_lap(lap)
    steering = np.asarray(lap.front_steer_deg, dtype=float)
    if steering.shape != (len(lap.travels), 2) or not np.all(np.isfinite(steering)):
        raise ValueError("Caster 最佳化需要 CSV 每筆資料完整包含 " + "、".join(FRONT_STEER_COLUMNS) + "。")
    if lap.turn_column not in TURN_COLUMNS:
        raise ValueError("Caster 最佳化需要 curvature_1_m 或 ay_g 以辨識外側輪。")
    if sum(pose.turn_signal != 0 for pose in lap.poses) < 2:
        raise ValueError("Caster 最佳化至少需要兩筆彎道樣本，才能建立常態分布。")
    return lap


def _caster_mapping(project, lap, cache, cancelled):
    engine = Engine(project)
    if any(check["fallback"] for checks in engine.scene.geometry.get("mount_position_checks", {}).values()
           for check in checks.values() if check["requested_x"] is not None):
        raise ValueError("A 臂車身 X 超出候選幾何的有效範圍。")
    mapping = calculate_lap_camber(engine, "front", lap, cancelled, include_steering=True,
                                    _wheel_center_cache=cache)
    if mapping is None:
        raise SearchCancelled()
    return mapping


def _fit_camber(mapping, caster_deg):
    values = [row[OUTER_STEERED_CAMBER] for row in mapping["rows"]]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("外側輪 Camber 含無效數值。")
    curve = normal_camber_curve(values, OUTER_STEERED_CAMBER, "#fbbf24", False)
    if not curve["x"]:
        raise ValueError("外側輪無法建立常態分布："+curve["reason"])
    return CasterCamberFit(caster_deg, curve["mean_deg"], curve["std_deg"], curve["count"], min(values), max(values))


def search_front_caster(project, lap, *, cancelled=lambda: False, progress=lambda *args: None):
    """Evaluate the full grid and minimize |mu| on an identical valid sample set.

    Reuse the authoritative plot calculation. Caster alone changes joint X,
    so cached FV wheel-center poses remain valid for every candidate. Straight
    and unreachable samples follow the plot's existing exclusion rules; no
    candidate may improve its score by retaining a different subset.
    """
    project = validate(project)
    if enabled(project):
        raise ValueError("硬點模式的 Caster 為反算結果；請切換至參數模式後搜尋。")
    validate_caster_lap(lap)
    candidate = copy.deepcopy(project)
    values = caster_candidates()
    cache, rejected = {}, {}
    best, before, best_mapping, best_project, expected_samples = None, None, None, None, None
    if cancelled():
        raise SearchCancelled()
    try:
        baseline = _caster_mapping(candidate, lap, cache, cancelled)
        expected_samples = tuple(row["sample"] for row in baseline["rows"])
        before = _fit_camber(baseline, candidate["front"]["caster_deg"])
    except (ValueError, ArithmeticError):
        pass
    for tested, caster in enumerate(values, 1):
        if cancelled():
            raise SearchCancelled()
        candidate["front"]["caster_deg"] = caster
        try:
            mapping = _caster_mapping(candidate, lap, cache, cancelled)
            samples = tuple(row["sample"] for row in mapping["rows"])
            if expected_samples is None:
                expected_samples = samples
            elif samples != expected_samples:
                raise ValueError("候選的有效彎道樣本集合改變，無法公平比較。")
            fit = _fit_camber(mapping, caster)
        except (ValueError, ArithmeticError) as exc:
            reason = str(exc)
            rejected[reason] = rejected.get(reason, 0)+1
            progress(tested, len(values), caster, None, reason)
            continue
        if cancelled():
            raise SearchCancelled()
        progress(tested, len(values), caster, fit, "")
        if best is None or fit.error_deg < best.error_deg-1e-12:
            best, best_mapping, best_project = fit, mapping, copy.deepcopy(candidate)
    if cancelled():
        raise SearchCancelled()
    return CasterSearchResult(best_project, best, before, best_mapping, len(values), len(values), rejected)
