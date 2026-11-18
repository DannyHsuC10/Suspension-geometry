"""Discrete front/rear SVSA selection at one prescribed pure-heave pose."""
import copy
from dataclasses import dataclass
import math

from .engine import Engine, check_pose_travel
from .hardpoints import enabled
from .project import travel_limits, validate
from .roll_center_search import SearchCancelled


SVSA_STEP_M = .5
SVSA_MAX_M = 15.
ANGLE_TOLERANCE_DEG = 1e-9


@dataclass(frozen=True)
class SVSASearchSettings:
    heave_mm: float = 7.
    front_min_change_deg: float = .15
    rear_max_change_deg: float = .1

    def validate(self, project):
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in vars(self).values()):
            raise ValueError("SVSA 搜尋的三個設定都必須是有限數值。")
        if min(self.front_min_change_deg, self.rear_max_change_deg) < 0:
            raise ValueError("Caster 絕對變化量的下限／上限不得為負。")
        check_pose_travel((self.heave_mm, self.heave_mm), travel_limits(project))
        return self


@dataclass(frozen=True)
class CasterChange:
    svsa_m: float
    zero_deg: float
    target_deg: float

    @property
    def signed_deg(self):
        return self.target_deg-self.zero_deg

    @property
    def absolute_deg(self):
        return abs(self.signed_deg)


@dataclass(frozen=True)
class AxleSVSAResult:
    best: CasterChange | None
    tested: int
    total: int
    stop_reason: str
    stopped_svsa_m: float | None = None
    stopped_change: CasterChange | None = None
    geometry_rejected: int = 0
    error: str = ""


@dataclass(frozen=True)
class SVSASearchResult:
    project: dict | None
    front: AxleSVSAResult
    rear: AxleSVSAResult
    error: str = ""


def svsa_candidates(axle):
    if axle not in ("front", "rear"):
        raise ValueError("SVSA 搜尋只適用於前／後軸。")
    # Zero length is a degenerate instant center, not a buildable candidate.
    first = 2 if axle == "front" else 1
    return [index*SVSA_STEP_M for index in range(first, int(SVSA_MAX_M/SVSA_STEP_M)+1)]


def _check_mounts(engine, axle):
    for check in engine.scene.geometry.get("mount_position_checks", {}).get(axle, {}).values():
        if check["requested_x"] is not None and check["fallback"]:
            raise ValueError("A 臂車身 X 超出候選幾何的有效範圍")


def _caster_change(project, axle, length, heave_mm):
    candidate = copy.deepcopy(project)
    candidate[axle]["svsa"] = length
    engine = Engine(candidate)
    _check_mounts(engine, axle)
    motion = engine.sv[axle]
    values = []
    # Both axle contacts have the same prescribed height in pure heave, so
    # the road stays horizontal. Solve each axle independently: an unreachable
    # old companion axle must not invalidate this axle's candidate pose.
    for travel in (0., heave_mm):
        state = motion.side(travel)
        vector = state["kt"]-state["kg"]
        caster = motion.template.direction_sign*math.degrees(math.atan2(vector[0], vector[1]))
        if not math.isfinite(caster):
            raise ValueError("目標姿態的 Caster 無有效數值。")
        values.append(caster)
    return CasterChange(length, *values)


def _search_axle(project, axle, settings, cancelled, progress):
    lengths = svsa_candidates(axle)
    best, rejected, last_error = None, 0, ""
    for tested, length in enumerate(lengths, 1):
        if cancelled():
            raise SearchCancelled()
        try:
            change = _caster_change(project, axle, length, settings.heave_mm)
        except (ValueError, ArithmeticError) as exc:
            rejected += 1
            last_error = str(exc)
            progress(axle, tested, len(lengths), length, None, last_error)
            if cancelled():
                raise SearchCancelled()
            if axle == "front":
                return AxleSVSAResult(best, tested, len(lengths), "geometry", length,
                                      geometry_rejected=rejected, error=last_error)
            continue
        passed = (change.absolute_deg+ANGLE_TOLERANCE_DEG >= settings.front_min_change_deg if axle == "front"
                  else change.absolute_deg <= settings.rear_max_change_deg+ANGLE_TOLERANCE_DEG)
        progress(axle, tested, len(lengths), length, change, "合格" if passed else "Caster 變化量未達條件")
        if cancelled():
            raise SearchCancelled()
        if axle == "front":
            if not passed:
                return AxleSVSAResult(best, tested, len(lengths), "caster_limit", length, change)
            best = change
        elif passed:
            return AxleSVSAResult(change, tested, len(lengths), "found", geometry_rejected=rejected)
    return AxleSVSAResult(best, len(lengths), len(lengths), "upper_limit" if axle == "front" else "no_solution",
                          geometry_rejected=rejected, error=last_error)


def search_svsa(project, settings, *, cancelled=lambda: False, progress=lambda *args: None):
    """Select both lengths, verify the combined pose, then return one snapshot.

    Front stops at the first failed candidate; rear may skip invalid shorter
    geometries to find its first valid candidate within the angle ceiling.
    Source inputs, Anti, initial caster and stroke limits are never mutated.
    """
    project = validate(project)
    if enabled(project):
        raise ValueError("硬點模式的 SVSA 為反算結果；請切換至參數模式後搜尋。")
    settings.validate(project)
    front = _search_axle(project, "front", settings, cancelled, progress)
    rear = _search_axle(project, "rear", settings, cancelled, progress)
    if cancelled():
        raise SearchCancelled()
    if front.best is None or rear.best is None:
        return SVSASearchResult(None, front, rear, "前後軸必須都找到合格值，才會一起套用。")
    candidate = copy.deepcopy(project)
    candidate["front"]["svsa"] = front.best.svsa_m
    candidate["rear"]["svsa"] = rear.best.svsa_m
    try:
        engine = Engine(candidate)
        for axle in ("front", "rear"):
            _check_mounts(engine, axle)
        zero = engine.solve_sv(0., 0.)["metrics"]
        target = engine.solve_sv(settings.heave_mm, settings.heave_mm)["metrics"]
        changes = {axle: CasterChange(candidate[axle]["svsa"], zero[f"caster_{axle}_deg"],
                                     target[f"caster_{axle}_deg"]) for axle in ("front", "rear")}
        if (not all(math.isfinite(change.absolute_deg) for change in changes.values())
                or changes["front"].absolute_deg+ANGLE_TOLERANCE_DEG < settings.front_min_change_deg
                or changes["rear"].absolute_deg > settings.rear_max_change_deg+ANGLE_TOLERANCE_DEG):
            raise ValueError("組合姿態的 Caster 變化量未達設定條件。")
    except (ValueError, ArithmeticError) as exc:
        return SVSASearchResult(None, front, rear, f"前後軸組合驗證失敗：{exc}")
    if cancelled():
        raise SearchCancelled()
    return SVSASearchResult(copy.deepcopy(engine.project), front, rear)
