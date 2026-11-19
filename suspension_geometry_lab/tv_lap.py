"""Fit an equivalent rack stroke for each recorded LapSim wheel-angle pair."""

import math

import numpy as np

from .steering import ackermann_metrics
from .tv_analysis import CAMBER_FIELDS, SIDES


def _camber_at_stroke(rows, stroke):
    """Interpolate static model camber at a matched, reachable rack stroke."""
    index = int(np.searchsorted([row["rack_travel_mm"] for row in rows], stroke))
    upper = min(index, len(rows) - 1)
    lower = max(0, upper - 1)
    if rows[upper]["rack_travel_mm"] == stroke:
        lower = upper
    start, end = rows[lower], rows[upper]
    span = end["rack_travel_mm"] - start["rack_travel_mm"]
    ratio = (stroke - start["rack_travel_mm"]) / span if span else 0.
    metrics = {}
    for side in SIDES:
        for field in CAMBER_FIELDS:
            key = f"{side}_{field}"
            first, last = start.get(key), end.get(key)
            metrics[key] = ((1. - ratio) * first + ratio * last
                            if start["valid"] and end["valid"] and first is not None and last is not None
                            and math.isfinite(first) and math.isfinite(last) else None)
    return metrics


def _pair_projection(rows):
    """Build a nearest-polyline projection without bridging unsolved strokes."""
    if not rows:
        return None
    travel = np.array([row["rack_travel_mm"] for row in rows], dtype=float)
    headings = np.array([(row["left_steer_deg"], row["right_steer_deg"]) for row in rows], dtype=float)
    valid = (np.array([row["valid"] for row in rows], dtype=bool)
             & np.isfinite(travel) & np.isfinite(headings).all(axis=1))
    change = np.diff(headings, axis=0)
    length_squared = np.sum(change * change, axis=1)
    # A principal heading wrap is a discontinuity in the reported angle pair.
    continuous = (np.abs(change) <= 180.).all(axis=1)
    connected = valid[:-1] & valid[1:] & continuous & (length_squared > 0.)

    if not connected.any():
        fixed = np.flatnonzero(valid)
        if fixed.size != 1:
            return None
        index = fixed[0]

        def project_fixed(pair):
            error = math.dist(pair, headings[index])
            return (float(travel[index]), error) if error <= 1e-9 else None

        return project_fixed

    starts = headings[:-1][connected]
    vectors = change[connected]
    lengths = length_squared[connected]
    start_travel = travel[:-1][connected]
    end_travel = travel[1:][connected]
    first_segment = np.r_[True, ~connected[:-1]][connected]
    last_segment = np.r_[~connected[1:], True][connected]

    def project(pair):
        offsets = pair - starts
        fractions = np.sum(offsets * vectors, axis=1) / lengths
        clipped = np.clip(fractions, 0., 1.)
        residuals = offsets - clipped[:, None] * vectors
        errors_squared = np.sum(residuals * residuals, axis=1)
        index = np.argmin(errors_squared)
        # A pair beyond a segment chain's end cannot be assigned a rack stroke
        # inside that chain. An interior mismatch only changes the residual.
        if ((first_segment[index] and fractions[index] < -1e-9)
                or (last_segment[index] and fractions[index] > 1. + 1e-9)):
            return None
        ratio = clipped[index]
        stroke = (1. - ratio) * start_travel[index] + ratio * end_travel[index]
        return float(stroke), math.sqrt(errors_squared[index])

    return project


def calculate_tv_lap_overlay(engine, axle, lap, sweep):
    """Keep measured wheel angles and infer rack stroke from the TV sweep.

    The LapSim CSV records wheel headings in degrees. Each matched stroke is
    the signed position in mm whose model left/right heading pair is closest
    to the measured pair. `fit_error_deg` is their Euclidean separation.
    """
    if axle not in ("front", "rear"):
        raise ValueError("請選擇前軸或後軸。")
    if axle == "front":
        angles, labels = lap.front_steer_deg, "steer_FL_deg／steer_FR_deg"
    else:
        angles, labels = lap.rear_steer_deg, "steer_RL_deg／steer_RR_deg"
    rows = []
    reason = None
    if not angles:
        reason = f"LapSim CSV 缺少 {labels} 左右輪轉角欄位。"
    else:
        project = _pair_projection(sweep["rows"])
        if project is None:
            reason = "目前的 Rack 掃描沒有可用於配對的左右輪轉角。"
        else:
            vehicle = engine.project["vehicle"]
            track = vehicle[f"track_{axle}"]
            wheelbase = vehicle["wheelbase"]
            for sample, pair in enumerate(angles, start=1):
                if pair is None or not all(-180. <= angle <= 180. for angle in pair):
                    continue
                match = project(np.asarray(pair, dtype=float))
                if match is None:
                    continue
                stroke, error = match
                left, right = pair
                ackermann = ackermann_metrics(axle, left, right, track, wheelbase)
                rows.append({"sample": sample, "rack_travel_mm": stroke, "left_steer_deg": left,
                             "right_steer_deg": right, "ackermann_pct": ackermann["ackermann_pct"],
                             "fit_error_deg": error, **_camber_at_stroke(sweep["rows"], stroke)})
            if not rows:
                reason = "LapSim 左右輪轉角在目前的 Rack 掃描範圍內沒有可配對工況。"

    return {"rows": rows, "valid": len(rows), "total": len(lap.travels), "source": "wheel_angles", "reason": reason}
