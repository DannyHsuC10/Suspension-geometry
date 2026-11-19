"""Static 3D rack and tie-rod kinematics for the desktop top view."""

import math

import numpy as np
from numpy.polynomial import Polynomial

from .engine import Unreachable
from .project import check_rack_travel


UP = np.array([0., 0., 1.])
EPS = np.finfo(float).eps
TIRE_ANGLES = np.linspace(0., math.tau, 65)
TIRE_DIRECTIONS = np.column_stack((np.cos(TIRE_ANGLES), np.sin(TIRE_ANGLES), np.zeros_like(TIRE_ANGLES)))


def _rack_rotation(corner, radial, tangent, offset, travel):
    """Follow the rod closure branch that contains the centered rack pose."""
    if travel == 0.:
        return 0.

    # With rack displacement t*travel, closure is a(t) cos(angle) +
    # b(t) sin(angle) = c(t). At t=0 the specified rod length makes c(0)=a(0).
    a0 = float(radial @ offset)
    b0 = float(tangent @ offset)
    if abs(b0) <= 64. * EPS * np.linalg.norm(radial) * np.linalg.norm(offset):
        raise Unreachable(f"{corner} 的拉桿在齒條零位處於死點，無法判定轉向分支。")
    a = Polynomial([a0, radial[1] * travel])
    b = Polynomial([b0, tangent[1] * travel])
    difference = Polynomial([0., (offset[1] - radial[1]) * travel, .5 * travel**2])
    c = a + difference

    def closure(fraction):
        a_value, b_value, c_value = a(fraction), b(fraction), c(fraction)
        reach = math.hypot(a_value, b_value)
        discriminant = b_value**2 - difference(fraction) * (a_value + c_value)
        return discriminant, 64. * EPS * max(reach, abs(c_value))**2

    final_discriminant, tolerance = closure(1.)
    if final_discriminant < -tolerance:
        raise Unreachable(f"{corner} 的拉桿在齒條行程 {travel * 1000:g} mm 時無法閉合。")

    # A feasible endpoint may lie beyond a gap or an over-center position.
    # The quartic's interior minima detect both without a travel step size.
    discriminant = b * b - difference * (a + c)
    for critical in discriminant.deriv().roots():
        if abs(critical.imag) <= 64. * EPS and 0. < critical.real < 1.:
            value, tolerance = closure(critical.real)
            if value <= tolerance:
                raise Unreachable(f"{corner} 無法從齒條零位連續移到 {travel * 1000:g} mm，中途有死點或無解區間。")

    a1, b1 = a(1.), b(1.)
    if math.hypot(a1, b1) <= 64. * EPS * math.hypot(a0, b0):
        raise Unreachable(f"{corner} 的齒條位置無法決定唯一的轉向角。")
    branch = math.copysign(1., b0)
    root = math.sqrt(max(0., final_discriminant))
    phase_change = math.atan2(a0 * b1 - b0 * a1, a0 * a1 + b0 * b1)
    opening0 = math.atan2(abs(b0), a0)
    opening = math.atan2(root, c(1.))
    angle = phase_change - branch * (opening - opening0)

    # The rationalized half-angle root avoids subtracting nearly equal
    # angles for small inputs. The phase form preserves motion past half a turn.
    denominator = b1 + branch * root
    if abs(angle) < math.pi / 2. and abs(denominator) > 64. * EPS * abs(b0):
        return 2. * math.atan(difference(1.) / denominator)
    return angle


def _tire_outline(center, normal, radius, width):
    """Horizontal silhouette of the steered cylinder used in wheel-end view."""
    along_axle = TIRE_DIRECTIONS @ normal
    radial = TIRE_DIRECTIONS - along_axle[:, None] * normal
    radial_length = np.linalg.norm(radial, axis=1)
    radial = np.divide(radial, radial_length[:, None], out=np.zeros_like(radial),
                       where=radial_length[:, None] > 1e-12)
    outline = center + radius * radial + .5 * width * np.sign(along_axle)[:, None] * normal
    outline[-1] = outline[0]
    return outline


def _wheel_state(engine, corner, pose, angle, tie_point, tie_length):
    normal, center = pose["normal"], pose["center"]
    forward = np.cross(normal, UP)
    heading_length = np.linalg.norm(forward)
    if heading_length < 1e-10:
        raise Unreachable(f"{corner} 的輪胎平面近乎水平，無法定義上視圖轉向角。")
    kingpin = engine.scene.geometry["kingpins"][corner]
    return dict(center=center, forward=forward / heading_length,
                tire_outline=_tire_outline(center, normal, engine.car.free_radius, engine.car.tire_width),
                tie_point=tie_point, tie_length=tie_length,
                kingpin_origin=pose["origin"], kingpin_axis=pose["axis"],
                kingpin_upper=kingpin["UBJ"], kingpin_lower=kingpin["LBJ"],
                kingpin_deg=angle, steer_deg=math.degrees(math.atan2(forward[1], forward[0])))


def ackermann_metrics(axle, left_deg, right_deg, track, wheelbase):
    """Compare an axle's signed wheel headings to ideal single-axle Ackermann."""
    result = dict(left_minus_right_deg=left_deg - right_deg, angle_difference_deg=abs(left_deg - right_deg),
                  turn_direction=None, inner_corner=None, outer_corner=None,
                  inner_steer_deg=None, outer_steer_deg=None, inner_minus_outer_deg=None, ideal_outer_deg=None,
                  ideal_angle_difference_deg=None, ackermann_pct=None)
    if (left_deg * right_deg <= 0. or min(abs(left_deg), abs(right_deg)) <= 1e-9
            or max(abs(left_deg), abs(right_deg)) >= 90.):
        return result

    # Treat the other axle as unsteered. With its center behind the front
    # axle and ahead of the rear axle, a positive rear wheel angle turns right.
    left_turn = (left_deg > 0.) == (axle == "front")
    prefix = "F" if axle == "front" else "R"
    left_corner, right_corner = prefix + "L", prefix + "R"
    inner_corner, outer_corner = (left_corner, right_corner) if left_turn else (right_corner, left_corner)
    inner_deg, outer_deg = (left_deg, right_deg) if left_turn else (right_deg, left_deg)
    inner, outer = math.radians(abs(inner_deg)), math.radians(abs(outer_deg))
    track_ratio = track / wheelbase
    ideal_outer = math.atan2(math.sin(inner), math.cos(inner) + track_ratio * math.sin(inner))
    # cot(outer) − cot(inner), expressed without subtracting two large cots
    # near straight ahead. The ideal planar linkage gives track / wheelbase.
    ackermann = 100. * math.sin(inner - outer) / (math.sin(outer) * math.sin(inner) * track_ratio)
    result.update(turn_direction="left" if left_turn else "right", inner_corner=inner_corner,
                  outer_corner=outer_corner, inner_steer_deg=inner_deg, outer_steer_deg=outer_deg,
                  inner_minus_outer_deg=abs(inner_deg) - abs(outer_deg),
                  ideal_outer_deg=math.copysign(math.degrees(ideal_outer), inner_deg),
                  ideal_angle_difference_deg=abs(inner_deg) - math.degrees(ideal_outer),
                  ackermann_pct=ackermann)
    return result


def _steering_metrics(axle, wheels, track, wheelbase, travel_mm):
    left, right = wheels.values()
    left_deg, right_deg = left["steer_deg"], right["steer_deg"]
    return dict(rack_travel_mm=travel_mm, left_steer_deg=left_deg, right_steer_deg=right_deg,
                left_kingpin_deg=left["kingpin_deg"], right_kingpin_deg=right["kingpin_deg"],
                **ackermann_metrics(axle, left_deg, right_deg, track, wheelbase))


def solve_steering(engine, axle, settings, *, travel_mm=None):
    """Solve both wheels against one lateral rack at the static axle pose.

    Settings use vehicle +X forward, +Y left and +Z up in metres, except
    for signed rack travel in millimetres. The FV instant center and centered
    rack determine the upright heights and rod lengths at zero steering.
    A disabled rear mechanism stays centered regardless of the requested travel.
    """
    if axle not in ("front", "rear"):
        raise ValueError("請選擇前軸或後軸。")
    rack_center, neutral_pickups = engine.centered_steering_pickups(axle, settings)
    enabled = settings.get("enabled", False) if axle == "rear" else True
    if type(enabled) is not bool:
        raise ValueError("後輪轉向啟用設定須為布林值。")
    try:
        if not enabled:
            travel_mm = 0.
        elif travel_mm is None:
            travel_mm = settings["rack_travel_mm"]
        travel_mm = float(travel_mm)
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError("齒條行程須為有限數值（mm）。") from exc
    if not math.isfinite(travel_mm):
        raise ValueError("齒條行程須為有限數值（mm）。")
    check_rack_travel(settings, travel_mm)

    travel = travel_mm / 1000.
    rack_center[1] = travel
    rack_displacement = np.array([0., travel, 0.])
    wheels, wheels_neutral, rack_ends, rack_ends_neutral = {}, {}, {}, {}
    prefix = "F" if axle == "front" else "R"
    try:
        with np.errstate(over="raise", invalid="raise"):
            for suffix, (tie_point, rack_end0) in neutral_pickups.items():
                corner = prefix + suffix
                rack_end = rack_end0 + rack_displacement
                neutral = engine.wheel_end_pose(0., corner)
                axis, origin = neutral["axis"], neutral["origin"]
                center = origin + axis * ((tie_point - origin) @ axis)
                radial = tie_point - center
                tie_length = math.hypot(*(tie_point - rack_end0))
                if math.hypot(*radial) <= 1e-10 or tie_length <= 1e-10:
                    raise Unreachable(f"{corner} 的拉桿接點位於主銷軸上或與齒條端點重合。")
                angle = _rack_rotation(corner, radial, np.cross(axis, radial), rack_end0 - center, travel)
                steer_deg = math.degrees(angle)
                wheels_neutral[corner] = _wheel_state(engine, corner, neutral, 0., tie_point, tie_length)
                if angle == 0.:
                    steered_tie = tie_point
                    wheels[corner] = wheels_neutral[corner]
                else:
                    pose = engine.wheel_end_pose(steer_deg, corner)
                    steered_tie = center + pose["matrix"] @ radial
                    wheels[corner] = _wheel_state(engine, corner, pose, steer_deg, steered_tie, tie_length)
                if not math.isclose(math.hypot(*(steered_tie - rack_end)), tie_length,
                                    rel_tol=1e-8, abs_tol=1e-10):
                    raise Unreachable(f"{corner} 在齒條行程 {travel_mm:g} mm 時無法維持拉桿長度。")
                rack_ends_neutral[corner], rack_ends[corner] = rack_end0, rack_end
    except (FloatingPointError, OverflowError, np.linalg.LinAlgError) as exc:
        raise Unreachable("轉向幾何的座標或齒條行程過大，無法計算。") from exc

    vehicle = engine.project["vehicle"]
    metrics = _steering_metrics(axle, wheels, vehicle[f"track_{axle}"], vehicle["wheelbase"], travel_mm)
    metrics["tie_rod_z"] = float(neutral_pickups["L"][0][2])
    return dict(metrics=metrics, wheels=wheels, wheels_neutral=wheels_neutral,
                rack_ends=rack_ends, rack_ends_neutral=rack_ends_neutral, rack_center=rack_center)
