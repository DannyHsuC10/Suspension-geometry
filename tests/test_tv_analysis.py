"""Validate TV stroke coverage and the two camber gain definitions."""

import math
import unittest

from suspension_geometry_lab.engine import Engine
from suspension_geometry_lab.project import default_project, steering_settings
from suspension_geometry_lab.steering import solve_steering
from suspension_geometry_lab.tv_analysis import calculate_tv_sweep, steering_camber_metrics


class TVAnalysisTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        project = default_project()
        project["front"]["static_camber_deg"] = -2.
        cls.engine = Engine(project)
        cls.settings = steering_settings(project)

    def camber_metrics(self, axle, travel):
        state = solve_steering(self.engine, axle, self.settings[axle], travel_mm=travel)
        return steering_camber_metrics(self.engine, axle, state)

    def test_neutral_bump_camber_gain_matches_fv(self):
        for axle in ("front", "rear"):
            with self.subTest(axle=axle):
                fv = self.engine.fv[axle].solve(0., 0.)["metrics"]
                tv = self.camber_metrics(axle, 0.)
                for side in ("left", "right"):
                    self.assertAlmostEqual(tv[f"{side}_camber_deg"], fv[f"camber_{side}_deg"])
                    self.assertAlmostEqual(tv[f"{side}_camber_change_deg"], 0.)
                    self.assertAlmostEqual(tv[f"{side}_camber_gain_deg_per_mm"], fv[f"camber_gain_{side}_deg_per_mm"])

    def test_steering_camber_gain_uses_wheel_heading(self):
        state = solve_steering(self.engine, "front", self.settings["front"], travel_mm=20.)
        metrics = steering_camber_metrics(self.engine, "front", state)
        for side, corner, sign in (("left", "FL", 1.), ("right", "FR", -1.)):
            steer = state["wheels"][corner]["kingpin_deg"]
            headings, cambers = [], []
            for delta in (-.0001, .0001):
                x, y, z = self.engine.wheel_end_pose(steer + delta, corner)["normal"]
                headings.append(math.degrees(math.atan2(-x, y)))
                cambers.append(math.degrees(math.asin(-sign*z)))
            gain = (cambers[1] - cambers[0]) / (headings[1] - headings[0])
            self.assertAlmostEqual(metrics[f"{side}_steering_camber_gain_deg_per_deg"], gain, places=8)

    def test_bump_gains_change_with_solved_steering_and_mirror(self):
        positive = self.camber_metrics("front", 20.)
        negative = self.camber_metrics("front", -20.)
        left = "left_camber_gain_deg_per_mm"
        right = "right_camber_gain_deg_per_mm"
        self.assertNotAlmostEqual(positive[left], positive[right], places=6)
        self.assertAlmostEqual(positive[left], negative[right], places=8)
        self.assertAlmostEqual(positive[right], negative[left], places=8)

    def test_sweep_preserves_asymmetric_bounds_neutral_and_invalid_rows(self):
        settings = dict(self.settings["front"], rack_limit_left_mm=18., rack_limit_right_mm=100.)
        sweep = calculate_tv_sweep(self.engine, "front", settings, samples=15)
        rows = sweep["rows"]
        self.assertEqual(len(rows), 15)
        self.assertEqual(rows[0]["rack_travel_mm"], -100.)
        self.assertEqual(rows[-1]["rack_travel_mm"], 18.)
        self.assertFalse(rows[0]["valid"])
        self.assertIsNone(rows[0]["left_steer_deg"])
        self.assertIsNone(rows[0]["right_camber_gain_deg_per_mm"])
        self.assertTrue(rows[0]["error"])
        self.assertTrue(rows[-1]["valid"])
        self.assertEqual(sweep["valid"], sum(row["valid"] for row in rows))
        zero, = [row for row in rows if row["rack_travel_mm"] == 0.]
        self.assertTrue(zero["valid"])
        self.assertIsNone(zero["ackermann_pct"])

    def test_disabled_rear_sweep_evaluates_only_the_fixed_neutral_pose(self):
        sweep = calculate_tv_sweep(self.engine, "rear", self.settings["rear"])
        self.assertEqual(sweep["valid"], 1)
        self.assertEqual(sweep["total"], 1)
        row, = sweep["rows"]
        self.assertEqual(row["rack_travel_mm"], 0.)
        self.assertEqual(row["left_steer_deg"], 0.)
        self.assertEqual(row["right_steer_deg"], 0.)
        self.assertIsNone(row["ackermann_pct"])


if __name__ == "__main__":
    unittest.main()
