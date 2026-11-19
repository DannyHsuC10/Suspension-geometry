"""Shared wheel-end and steering linkage geometry, in metres and radians.

The fixed origin lies on the kingpin at initial wheel-center height.
Keep the original sus_3Dwe_v3 axis signs. At z=-tire_radius, positive
mechanical trail places the tire-bottom reference along +X from the
kingpin intersection; positive scrub places it along -Y (wheel outward).
These are the wheel-end/SV/FV signs, not steering_camber's two-wheel frame.
"""

import math

import numpy as np


def kingpin_axis(suspension):
    """Return the upward unit axis using the original v3 direction formula."""
    kpi = float(suspension.kpi)
    caster = np.deg2rad(float(suspension.caster_sv_deg))
    if (not np.all(np.isfinite([kpi, caster]))
            or abs(kpi) >= np.pi / 2.0 or abs(caster) >= np.pi / 2.0):
        raise ValueError("KPI and caster must be finite and strictly between -90 and 90 deg.")
    axis = np.array([
        np.sin(caster), np.sin(kpi), np.cos(kpi) * np.cos(caster),
    ])
    return axis / np.linalg.norm(axis)


def initial_wheel_center(suspension, tire_radius):
    """Offset the wheel so ground-plane trail/scrub equal the input lengths.

    Kingpin/ground intersection G = -radius * axis / axis_z.
    Initial tire-bottom P0 = G + [trail, -scrub, 0].
    Initial wheel center C0 = P0 + [0, 0, radius].
    Thus C0.z=0 but C0 generally does not lie on the kingpin.
    """
    radius = float(tire_radius)
    trail = float(suspension.mechanical_trail_sv)
    scrub = float(suspension.scrub_radius)
    if not np.all(np.isfinite([radius, trail, scrub])) or radius <= 0.0:
        raise ValueError("Tire radius must be positive; radius, trail and scrub must be finite.")
    axis = kingpin_axis(suspension)
    if axis[2] < 1e-10:
        raise ValueError("Kingpin is too close to horizontal to define ground offsets.")
    center = -radius * axis / axis[2] + np.array([trail, -scrub, radius])
    center[2] = 0.0
    return center


def centered_steering_pickups(settings, fv_ic):
    """Place the upright points on the rack-to-IC lines projected onto Y–Z."""
    try:
        tie_x, tie_y = (float(settings[f"tie_rod_{axis}"]) for axis in "xy")
        rack_x, rack_z, length = (float(settings[key]) for key in ("rack_x", "rack_z", "rack_length"))
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError("轉向拉桿接點、齒條位置與長度須為有限數值。") from exc
    if not np.isfinite([tie_x, tie_y, rack_x, rack_z, length]).all():
        raise ValueError("轉向拉桿接點、齒條位置與長度須為有限數值。")
    if tie_y <= 0. or length <= 0.:
        raise ValueError("左側拉桿接點 Y 與齒條長度須大於零。")
    if fv_ic is None or not np.isfinite(fv_ic).all():
        raise ValueError("前視 A 臂瞬心（IC）無法定義，無法求解拉桿接點 Z。")
    # The FV source is the right wheel, with FV horizontal coordinate -Y.
    # Mirroring it across the vehicle centerline makes that value left-side Y.
    ic_y, ic_z = map(float, fv_ic)
    rack_y = length / 2.
    if math.isclose(rack_y, ic_y, rel_tol=1e-12, abs_tol=1e-10):
        raise ValueError("齒條端點與前視瞬心的 Y 座標重合，無法唯一求解拉桿接點 Z。")
    if math.isclose(tie_y, rack_y, rel_tol=1e-12, abs_tol=1e-10):
        raise ValueError("拉桿外接點與齒條端點的 Y 座標重合，無法建立前視投影線。")
    tie_z = rack_z + ((tie_y - rack_y) / (ic_y - rack_y)) * (ic_z - rack_z)
    if not math.isfinite(tie_z):
        raise ValueError("前視共瞬心求得的拉桿接點 Z 無法表示為有限數值。")
    center = np.array([rack_x, 0., rack_z])
    return center, {
        suffix: (np.array([tie_x, side * tie_y, tie_z]), np.array([rack_x, side * rack_y, rack_z]))
        for suffix, side in (("L", 1.), ("R", -1.))
    }


def rotation_matrix(axis, theta):
    """Rodrigues rotation about an axis through the fixed origin."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    # The half-angle identity preserves the quadratic term near zero steer.
    return np.eye(3) + np.sin(theta) * cross + 2.0 * np.sin(theta / 2.0)**2 * (cross @ cross)
