"""LapSim operating points preserve measured steering while locating rack stroke."""

import csv
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from suspension_geometry_lab.lap_data import LapData, TRAVEL_COLUMNS, read_lap_data
from suspension_geometry_lab.tv_lap import calculate_tv_lap_overlay


def sweep_row(travel, pair):
    left, right = pair if pair is not None else (None, None)
    return {"rack_travel_mm": travel, "left_steer_deg": left, "right_steer_deg": right, "valid": pair is not None}


class TvLapTests(unittest.TestCase):
    engine = SimpleNamespace(project={"vehicle": {"track_front": 1.2, "track_rear": 1.1, "wheelbase": 1.5}})

    def test_projection_uses_both_angles_and_preserves_measured_ordinates(self):
        rows = [sweep_row(-10., (8., 6.)), sweep_row(0., (0., 0.)), sweep_row(10., (-8., -6.))]
        for row, left, right in zip(rows, (-2., -1., 3.), (2., -1., -3.)):
            row.update(left_camber_deg=left, right_camber_deg=right,
                       left_camber_gain_deg_per_mm=left / 100., right_camber_gain_deg_per_mm=right / 100.)
        samples = ((4., 3.), (4., 4.), (-10., -7.5), (8., 6.))
        lap = LapData(((0.,) * 4,) * len(samples), (), None, samples)
        overlay = calculate_tv_lap_overlay(self.engine, "front", lap, {"rows": rows})
        self.assertEqual((overlay["valid"], overlay["total"], overlay["source"], overlay["reason"]),
                         (3, 4, "wheel_angles", None))
        first, second, endpoint = overlay["rows"]
        self.assertEqual((first["rack_travel_mm"], first["fit_error_deg"]), (-5., 0.))
        self.assertAlmostEqual(first["left_camber_deg"], -1.5)
        self.assertAlmostEqual(first["right_camber_deg"], .5)
        self.assertAlmostEqual(first["left_camber_gain_deg_per_mm"], -.015)
        self.assertAlmostEqual(first["ackermann_pct"],
                               100. / .8 * (1. / math.tan(math.radians(3.)) - 1. / math.tan(math.radians(4.))))
        self.assertAlmostEqual(second["rack_travel_mm"], -5.6)
        self.assertAlmostEqual(second["fit_error_deg"], .8)
        self.assertEqual((second["left_steer_deg"], second["right_steer_deg"], second["ackermann_pct"]), (4., 4., 0.))
        self.assertEqual((endpoint["sample"], endpoint["rack_travel_mm"]), (4, -10.))

    def test_inference_does_not_bridge_an_unreachable_gap(self):
        rows = [sweep_row(t, pair) for t, pair in
                ((-10., (10., 10.)), (-5., (5., 5.)), (0., None), (5., (-5., -5.)), (10., (-10., -10.)))]
        lap = LapData(((0.,) * 4,) * 2, (), None, ((0., 0.), (7., 7.)))
        overlay = calculate_tv_lap_overlay(self.engine, "front", lap, {"rows": rows})
        self.assertEqual(overlay["valid"], 1)
        row, = overlay["rows"]
        self.assertEqual(row["sample"], 2)
        self.assertAlmostEqual(row["rack_travel_mm"], -7.)

    def test_fixed_rear_rack_only_matches_straight_ahead(self):
        lap = LapData(((0.,) * 4,) * 3, (), None, (), ((0., 0.), (.3, .3), None))
        overlay = calculate_tv_lap_overlay(self.engine, "rear", lap, {"rows": [sweep_row(0., (0., 0.))]})
        self.assertEqual((overlay["valid"], overlay["total"]), (1, 3))
        row, = overlay["rows"]
        self.assertEqual(row["rack_travel_mm"], 0.)
        self.assertIsNone(row["ackermann_pct"])
        missing = calculate_tv_lap_overlay(self.engine, "front", lap, {"rows": [sweep_row(0., (0., 0.))]})
        self.assertIn("steer_FL_deg", missing["reason"])

    def test_csv_retains_rear_measurements_and_keeps_incomplete_rows_aligned(self):
        directory = Path(__file__).resolve().parents[1] / ".test-tmp"
        directory.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=directory) as temporary:
            path = Path(temporary) / "lap.csv"
            with path.open("w", encoding="utf-8-sig", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([*TRAVEL_COLUMNS, "steer_FL_deg", "steer_FR_deg", "steer_RL_deg", "steer_RR_deg"])
                writer.writerows(((1, 2, 3, 4, 5, 6, 7, 8), (1, 2, 3, 4, -5, -6, "", ""), (1, 2, 3, 4, 0, 0, 0, 0)))
            lap = read_lap_data(path)
        self.assertEqual(lap.travels, ((-1., -2., -3., -4.),) * 3)
        self.assertEqual(lap.front_steer_deg, ((5., 6.), (-5., -6.), (0., 0.)))
        self.assertEqual(lap.rear_steer_deg, ((7., 8.), None, (0., 0.)))
