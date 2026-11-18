"""Front RC search and direct analytical rear RC with full-stroke validation.

Heights use the current contact line as ground, just like FVMotion.solve.
Finite sampling cannot certify the continuous mechanism between samples.
"""
import copy
from dataclasses import dataclass
from decimal import Decimal, ROUND_CEILING, ROUND_FLOOR
import math

import numpy as np

from .engine import Engine, Unreachable
from .hardpoints import enabled
from .input_limits import AXLE_LIMITS, INPUT_STEPS, round_to_step
from .project import validate
from .roll_balance import AnalyticRearRC, LoadTransferBalance, analytic_rear_solution, balance_from_engine


class SearchCancelled(Exception):
    pass


@dataclass(frozen=True)
class SearchSettings:
    height_min_mm: float
    height_max_mm: float
    droop_mm: float
    bump_mm: float
    clearance_mm: float = 0.

    def validate(self, *, require_grid=True):
        if any(type(value) not in (int, float) or not math.isfinite(value)
               for value in vars(self).values()):
            raise ValueError("搜尋設定必須是有限數值。")
        limit = AXLE_LIMITS["rc_height"]
        if not (limit.low*1000 <= self.height_min_mm <= self.height_max_mm <= limit.high*1000):
            raise ValueError(f"RC 搜尋高度須介於 {limit.low*1000:g}～{limit.high*1000:g} mm，且下限不得大於上限。")
        if not (0 <= self.droop_mm <= 200 and 0 <= self.bump_mm <= 200
                and self.droop_mm+self.bump_mm > 0):
            raise ValueError("上壓／下垂行程各須為 0～200 mm，且不可同時為 0。")
        if self.clearance_mm < 0:
            raise ValueError("最低離地餘裕不可為負。")
        if require_grid and not self.height_candidates():
            raise ValueError("高度範圍內沒有符合 RC 設定級距的候選值。")
        return self

    def height_candidates(self):
        step = Decimal(str(INPUT_STEPS["rc_height"]))
        low = Decimal(str(self.height_min_mm))/1000
        high = Decimal(str(self.height_max_mm))/1000
        first = int((low/step).to_integral_value(rounding=ROUND_CEILING))
        last = int((high/step).to_integral_value(rounding=ROUND_FLOOR))
        return [float(i*step) for i in range(first, last+1)]

    def travels(self):
        # Retain both stroke endpoints and zero; intervals never exceed the
        # current minimum RC design increment (1 mm at present).
        step_mm = INPUT_STEPS["rc_height"]*1000
        return np.unique(np.concatenate((
            np.linspace(-self.droop_mm, 0., max(1, math.ceil(self.droop_mm/step_mm))+1),
            np.linspace(0., self.bump_mm, max(1, math.ceil(self.bump_mm/step_mm))+1))))


@dataclass(frozen=True)
class TravelCheck:
    valid: bool
    reason: str = ""
    min_height_mm: float | None = None
    max_abs_x_mm: float | None = None
    worst_travel_mm: tuple[float, float] | None = None


@dataclass(frozen=True)
class SearchResult:
    project: dict | None
    check: TravelCheck | None
    tested: int
    total: int
    rejected: dict
    travel_samples: int
    balance: LoadTransferBalance | None = None
    reference_balance: LoadTransferBalance | None = None
    front_check: TravelCheck | None = None
    travel_checked: int = 0
    analytic: AnalyticRearRC | None = None
    rounded_height_m: float | None = None
    failure_reason: str = ""


def _cross(first, second):
    return first[..., 0]*second[..., 1]-first[..., 1]*second[..., 0]


def check_travel(motion, travels, clearance_mm=0., cancelled=lambda: False):
    """Reuse each one-wheel solve, then intersect all left/right force lines.

    Detect determinant sign changes as well as undefined sampled intersections:
    a pole between samples can otherwise masquerade as two finite RC positions.
    """
    states = []
    arm_determinants = []
    for travel in travels:
        if cancelled():
            raise SearchCancelled()
        try:
            state = motion.side(float(travel))
        except (Unreachable, ArithmeticError, ValueError):
            return TravelCheck(False, "行程內存在不可達姿態")
        if state["IC"] is None:
            return TravelCheck(False, "行程內瞬心無有限交點")
        upper = state["kua"]-state["cpb_ru"]
        lower = state["lak"]-state["cpb_rl"]
        scale = np.linalg.norm(upper)*np.linalg.norm(lower)
        if scale <= 1e-12:
            return TravelCheck(False, "行程內 A 臂退化")
        arm_determinants.append(float(_cross(upper, lower)/scale))
        states.append(state)
    arm_determinants = np.asarray(arm_determinants)
    if (not np.all(np.isfinite(arm_determinants)) or np.any(np.abs(arm_determinants) <= 1e-10)
            or np.min(arm_determinants) < 0 < np.max(arm_determinants)):
        return TravelCheck(False, "行程內瞬心通過平行奇異位置")
    right = np.asarray([s.get("contact", s["gnd"]) for s in states])
    directions = np.asarray([s["IC"]-s.get("contact", s["gnd"]) for s in states])
    lengths = np.linalg.norm(directions, axis=1)
    if (not np.all(np.isfinite(right)) or not np.all(np.isfinite(directions))
            or np.any(lengths <= 1e-12)):
        return TravelCheck(False, "接地點／瞬心無法定義有效力線")
    directions = directions/lengths[:, None]
    left = right*np.array([-1., 1.])
    dl = (directions*np.array([-1., 1.]))[:, None, :]
    dr = directions[None, :, :]
    ground = right[None, :, :]-left[:, None, :]
    spans = np.linalg.norm(ground, axis=2)
    determinant = _cross(dl, dr)
    if (not np.all(np.isfinite(determinant)) or np.any(spans <= 1e-12)
            or np.any(np.abs(determinant) <= 1e-10)
            or np.min(determinant) < 0 < np.max(determinant)):
        return TravelCheck(False, "行程內 RC 力線平行或跨越奇異位置")
    # Relative to the left contact: avoids subtracting a large world origin.
    offset = (_cross(ground, dr)/determinant)[..., None]*dl
    heights = _cross(ground, offset)/spans*1000
    horizontal = np.sum((offset-ground/2)*ground, axis=2)/spans*1000
    if not np.all(np.isfinite(heights)) or not np.all(np.isfinite(horizontal)):
        return TravelCheck(False, "行程內 RC 無有限位置")
    i, j = np.unravel_index(np.argmin(heights), heights.shape)
    minimum = float(heights[i, j])
    return TravelCheck(minimum > clearance_mm+1e-6,
                       "" if minimum > clearance_mm+1e-6 else "RC 觸地、低於地面或離地餘裕不足",
                       minimum, float(np.max(np.abs(horizontal))),
                       (float(travels[i]), float(travels[j])))


def search_roll_center(project, axle, settings, *, cancelled=lambda: False,
                       progress=lambda tested, total, height_mm, reason: None):
    """Return the first feasible design in ascending, globally aligned RC steps.

    Never mutate the source project or accept only the reachable subset of a
    requested stroke range. The returned project includes that tested range.
    """
    settings.validate()
    project = validate(project)
    if axle not in ("front", "rear"):
        raise ValueError("RC 搜尋只適用於前／後軸 FV。")
    if enabled(project):
        raise ValueError("硬點模式的 RC 為反算結果；請切換至參數模式後搜尋。")
    candidate = copy.deepcopy(project)
    candidate["analysis"].update(droop_mm=settings.droop_mm, bump_mm=settings.bump_mm)
    heights, travels = settings.height_candidates(), settings.travels()
    rejected = {}
    for tested, height in enumerate(heights, 1):
        if cancelled():
            raise SearchCancelled()
        candidate[axle]["rc_height"] = height
        try:
            engine = Engine(candidate)
            if any(check["fallback"] for checks in engine.scene.geometry.get("mount_position_checks", {}).values()
                   for check in checks.values() if check["requested_x"] is not None):
                check = TravelCheck(False, "A 臂車身 X 超出候選幾何的有效範圍")
            else:
                check = check_travel(engine.fv[axle], travels, settings.clearance_mm, cancelled)
        except (ValueError, ArithmeticError) as exc:
            check = TravelCheck(False, f"靜態幾何無解：{exc}")
        if cancelled():
            raise SearchCancelled()
        progress(tested, len(heights), height*1000, check.reason)
        if check.valid:
            return SearchResult(copy.deepcopy(engine.project), check, tested, len(heights), rejected, len(travels))
        rejected[check.reason] = rejected.get(check.reason, 0)+1
    return SearchResult(None, None, len(heights), len(heights), rejected, len(travels))


def search_balanced_rear_roll_center(project, settings, *, cancelled=lambda: False,
                                     progress=lambda tested, total, height_mm, reason: None):
    """Directly solve ΔFz_front == ΔFz_rear, then validate one rounded design.

    The existing entry point is retained for callers. There is no height
    enumeration, clipping to bounds, or fallback to another feasible height.
    """
    settings.validate(require_grid=False)
    project = validate(project)
    if enabled(project):
        raise ValueError("硬點模式的 RC 為反算結果；請切換至參數模式後搜尋。")
    candidate = copy.deepcopy(project)
    candidate["analysis"].update(droop_mm=settings.droop_mm, bump_mm=settings.bump_mm)
    travels = settings.travels()
    front_check, reference = None, None
    travel_checked = 0
    if cancelled():
        raise SearchCancelled()
    analytic = analytic_rear_solution(candidate)
    height = round_to_step(analytic.rear_design_m, INPUT_STEPS["rc_height"])
    progress(1, 3, height*1000, f"解析解設計高度 {analytic.rear_design_m:.9g} m；取整後 {height:g} m")
    if cancelled():
        raise SearchCancelled()

    def failed(reason, check=None):
        return SearchResult(None, check, 1, 1, {reason: 1}, len(travels),
                            reference_balance=reference, front_check=front_check, travel_checked=travel_checked,
                            analytic=analytic, rounded_height_m=height, failure_reason=reason)

    low, high = settings.height_min_mm/1000, settings.height_max_mm/1000
    if not low-1e-12 <= analytic.rear_design_m <= high+1e-12:
        return failed(f"解析解設計高度 {analytic.rear_design_m:.9g} m 超出允許界線 {low:g}～{high:g} m。")
    if not low-1e-12 <= height <= high+1e-12:
        return failed(f"取整後設計高度 {height:g} m 超出允許界線 {low:g}～{high:g} m。")
    try:
        reference = balance_from_engine(Engine(candidate))
    except (ValueError, ArithmeticError):
        # An invalid current rear draft must not prevent its analytical repair.
        pass
    if cancelled():
        raise SearchCancelled()
    candidate["rear"]["rc_height"] = height
    try:
        engine = Engine(candidate)
        if any(check["fallback"] for checks in engine.scene.geometry.get("mount_position_checks", {}).values()
               for check in checks.values() if check["requested_x"] is not None):
            return failed("A 臂車身 X 超出解析解幾何的有效範圍。")
        balance = balance_from_engine(engine)
    except (ValueError, ArithmeticError) as exc:
        return failed(f"解析解的幾何／平衡模型無效：{exc}")
    front_check = check_travel(engine.fv["front"], travels, settings.clearance_mm, cancelled)
    progress(2, 3, height*1000, "固定前軸全行程檢查："+(front_check.reason or "通過"))
    if cancelled():
        raise SearchCancelled()
    if not front_check.valid:
        return failed(f"固定前軸未通過全行程檢查：{front_check.reason}。請先調整前軸或執行前軸 RC 搜尋。")
    check = check_travel(engine.fv["rear"], travels, settings.clearance_mm, cancelled)
    travel_checked = 1
    progress(3, 3, height*1000, "解析解後軸全行程檢查："+(check.reason or "通過"))
    if cancelled():
        raise SearchCancelled()
    if not check.valid:
        return failed(f"解析解未通過後軸全行程檢查：{check.reason}。", check)
    return SearchResult(copy.deepcopy(engine.project), check, 1, 1, {}, len(travels),
                        balance, reference, front_check, travel_checked, analytic, height)
