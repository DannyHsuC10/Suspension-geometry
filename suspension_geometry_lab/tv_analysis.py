"""Rack stroke sweeps and local camber measures for the TV page."""

from dataclasses import dataclass
import math

import numpy as np

from .engine import Unreachable, contact_frame
from .lap_data import road_camber
from .project import rack_travel_bounds
from .static.fv.wireframe_geometry import project_fv
from .steering import solve_steering
from .wheel_alignment import loaded_contact
from .wheel_end_geometry import rotation_matrix


DEFAULT_TV_SAMPLES = 121
FV_STEP_MM = .01  # The same local difference interval as the FV solve.
SIDES = ("left", "right")
CAMBER_FIELDS = ("camber_deg", "camber_change_deg", "steering_camber_gain_deg_per_deg", "camber_gain_deg_per_mm")


@dataclass(frozen=True)
class _BumpPose:
    center: np.ndarray
    pivot: np.ndarray
    kingpin: np.ndarray
    normal: np.ndarray


def _fv_bump_samples(engine, axle):
    """Prepare each corner at nominal stroke and at the adjacent FV samples."""
    samples = []
    fv = engine.fv[axle]
    for suffix in "LR":
        corner = axle[0].upper() + suffix
        neutral = engine.wheel_end_pose(0., corner)
        lower_x = engine.scene.geometry["kingpins"][corner]["LBJ"][0]
        poses = []
        for travel in (0., -FV_STEP_MM, FV_STEP_MM):
            try:
                state = fv.side(travel, left=suffix == "L")
            except Unreachable:
                poses.append(None)
                continue
            # FV rightward mirrors vehicle Y. Its upright rotation has the
            # opposite sign in the full vehicle coordinate system.
            motion = rotation_matrix((1., 0., 0.), -state["angle"])
            poses.append(_BumpPose(project_fv(state["wheel_center"], neutral["center0"][0]),
                                   project_fv(state["lak"], lower_x), motion @ neutral["axis"],
                                   motion @ neutral["normal"]))
        samples.append(poses)
    return tuple(samples)


def _steered_bump_contact(pose, steer, radius):
    if pose is None:
        return None
    rotation = rotation_matrix(pose.kingpin, math.radians(steer))
    center = pose.pivot + rotation @ (pose.center - pose.pivot)
    normal = rotation @ pose.normal
    try:
        return normal, loaded_contact(center, normal, radius)
    except ValueError:
        return None


def _bump_camber_gains(samples, steers, radius):
    neutral_contacts = [_steered_bump_contact(poses[0], steer, radius) for poses, steer in zip(samples, steers)]
    gains = []
    for wheel, (poses, steer, sign) in enumerate(zip(samples, steers, (1., -1.))):
        cambers = []
        for pose in poses[1:]:
            own = _steered_bump_contact(pose, steer, radius)
            contacts = neutral_contacts.copy()
            contacts[wheel] = own
            if not all(contact is not None for contact in contacts):
                break
            # Measure camber in the same transverse contact frame as FV, while
            # allowing steering itself to move the tire center and contact.
            left, right = (point[1:] * [-1., 1.] for _, point in contacts)
            try:
                up_right, up_z = contact_frame(left, right)["y_axis"]
            except Unreachable:
                break
            cambers.append(road_camber(sign * own[0], (0., -up_right, up_z)))
        gains.append((cambers[1] - cambers[0]) / (2. * FV_STEP_MM) if len(cambers) == 2 else None)
    return gains


def _steering_camber_and_gain(normal, kingpin, sign):
    x, y, z = normal
    dx, dy, dz = np.cross(kingpin, normal)
    horizontal = math.hypot(x, y)
    camber = math.degrees(math.atan2(-sign * z, horizontal))
    # d(normal)/d(kingpin angle) = axis cross normal. The heading is
    # atan2(-x, y); using the ratio of derivatives avoids angle wrapping.
    dheading = (x * dy - y * dx) / horizontal**2
    gain = float(-sign * dz / (horizontal * dheading)) if abs(dheading) > 1e-10 else None
    return camber, gain


def steering_camber_metrics(engine, axle, state, *, _bump_samples=None):
    """Evaluate both wheel Cambers and two independent local derivatives.

    Actual Camber is measured against the static horizontal vehicle plane.
    Steering gain uses actual wheel heading (deg/deg) at fixed suspension
    height. Bump gain uses the FV transverse contact frame (deg/mm) and
    holds each wheel's solved kingpin angle while jouncing one side.
    """
    metrics = {}
    samples = _fv_bump_samples(engine, axle) if _bump_samples is None else _bump_samples
    steers = tuple(state["metrics"][f"{side}_kingpin_deg"] for side in SIDES)
    radius = engine.fv[axle].loaded_radius
    bump_gains = _bump_camber_gains(samples, steers, radius)
    static_camber = engine.project[axle]["static_camber_deg"]
    for side, suffix, sign, steer, bump_gain in zip(SIDES, "LR", (1., -1.), steers, bump_gains):
        corner = axle[0].upper() + suffix
        pose = engine.wheel_end_pose(steer, corner)
        camber, steer_gain = _steering_camber_and_gain(pose["normal"], pose["axis"], sign)
        metrics.update({f"{side}_camber_deg": camber,
                        f"{side}_camber_change_deg": camber - static_camber,
                        f"{side}_steering_camber_gain_deg_per_deg": steer_gain,
                        f"{side}_camber_gain_deg_per_mm": bump_gain})
    return metrics


def calculate_tv_sweep(engine, axle, settings, *, samples=DEFAULT_TV_SAMPLES):
    """Resolve the configured rack interval while retaining infeasible rows."""
    if axle not in ("front", "rear"):
        raise ValueError("請選擇前軸或後軸。")
    if not isinstance(samples, int) or samples < 3:
        raise ValueError("TV 齒條掃描至少需要三個樣本。")
    low, high = rack_travel_bounds(settings)
    if axle == "rear" and not settings.get("enabled", False):
        low = high = 0.
    if low == high:
        travels = (0.,)
    else:
        fractions = np.linspace(0., 1., samples)
        travel = low * (1. - fractions) + high * fractions
        if low < 0. < high:
            # Preserve both limits and the specified straight-ahead branch.
            travel[1 + int(np.argmin(np.abs(travel[1:-1])))] = 0.
        travels = map(float, travel)
    rows = []
    bump = None
    for travel_mm in travels:
        row = {"axle": axle, "rack_travel_mm": travel_mm}
        try:
            state = solve_steering(engine, axle, settings, travel_mm=travel_mm)
        except (ValueError, ArithmeticError) as error:
            row.update(valid=False, error=str(error), left_steer_deg=None, right_steer_deg=None, ackermann_pct=None)
            row.update({f"{side}_{key}": None for side in SIDES for key in CAMBER_FIELDS})
        else:
            if bump is None:
                bump = _fv_bump_samples(engine, axle)
            camber = steering_camber_metrics(engine, axle, state, _bump_samples=bump)
            row.update(state["metrics"], **camber, valid=True, error=None)
        rows.append(row)
    return {"rows": rows, "valid": sum(row["valid"] for row in rows), "total": len(rows)}
