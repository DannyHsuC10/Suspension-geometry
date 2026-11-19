"""TV charts use the current pose and LapSim angles independently of the model sweep."""

import unittest
from unittest.mock import Mock

from suspension_geometry_lab.charts import LabChart
from suspension_geometry_lab.steering_chart import WHEEL_COLORS, steering_sweep_chart


def row(travel, left, right, ackermann):
    return {"rack_travel_mm": travel, "left_steer_deg": left, "right_steer_deg": right,
            "ackermann_pct": ackermann}


class TVSweepChartTests(unittest.TestCase):
    def setUp(self):
        self.sweep = {"rows": [row(-20., None, None, None), row(-10., 7., 8., 87.),
                               row(0., 0., 0., None), row(10., -8., -7., 87.)]}
        self.lap = {"rows": [row(-10., 7.5, 12., 94.), row(0., 0., 0., None)], "total": 3}

    def test_camber_and_gain_curves_show_both_sides_and_operating_points(self):
        for plot, field in (("camber", "camber_deg"), ("camber_gain", "camber_gain_deg_per_mm"),
                            ("steering_camber_gain", "steering_camber_gain_deg_per_deg")):
            with self.subTest(plot=plot):
                for item in self.sweep["rows"]:
                    item.update({f"left_{field}": -.25, f"right_{field}": .12})
                self.lap["rows"][0].update({f"left_{field}": -.2, f"right_{field}": .1})
                current = {"rack_travel_mm": 2., f"left_{field}": -.1, f"right_{field}": .2}
                spec = steering_sweep_chart("front", plot, self.sweep, metrics=current, lap=self.lap)
                self.assertEqual([s["y"] for s in spec["series"]], [[-.25] * 4, [.12] * 4])
                self.assertEqual(spec["lap_points"], [[-10., -.2], [-10., .1]])
                self.assertEqual([m["position"] for m in spec["markers"]], [[2., -.1], [2., .2]])

    def test_wheel_curves_and_lap_samples_retain_their_own_angles(self):
        chart = steering_sweep_chart("front", "steer", self.sweep, metrics=row(-4., 2., 3., 60.), lap=self.lap)
        left, right = chart["series"]
        self.assertEqual(left["x"], [-20., -10., 0., 10.])
        self.assertEqual(left["y"], [None, 7., 0., -8.])
        self.assertEqual(right["y"], [None, 8., 0., -7.])
        self.assertEqual([marker["position"] for marker in chart["markers"]], [[-4., 2.], [-4., 3.]])
        self.assertEqual(chart["lap_points"], [[-10., 7.5], [-10., 12.], [0., 0.], [0., 0.]])
        self.assertEqual(chart["lap_colors"], list(WHEEL_COLORS) * 2)
        self.assertEqual((chart["lap_visible"], chart["lap_total"]), (2, 3))
        self.assertIn([-20., 7.], chart["fit_points"])
        self.assertIn([-10., 12.], chart["fit_points"])

    def test_ackermann_does_not_join_the_curve_across_straight_ahead(self):
        chart = steering_sweep_chart("rear", "ackermann", self.sweep, metrics=row(0., 0., 0., None), lap=self.lap)
        curve, ideal = chart["series"]
        self.assertEqual(curve["y"], [None, 87., None, 87.])
        self.assertEqual(ideal["y"], [100., 100.])
        self.assertEqual(chart["markers"], [])
        self.assertEqual(chart["current_status"], "目前阿克曼率未定義")
        self.assertEqual(chart["lap_points"], [[-10., 94.]])
        self.assertEqual((chart["lap_visible"], chart["lap_total"]), (1, 3))

    def test_plot_fits_measured_points_and_only_deduplicates_the_same_color(self):
        renderer = object.__new__(LabChart)
        renderer.canvas = Mock()
        renderer.canvas.winfo_width.return_value = 800
        renderer.canvas.winfo_height.return_value = 420
        self.lap["rows"].append(row(0., 0., 0., None))
        renderer.set(steering_sweep_chart("front", "steer", self.sweep, lap=self.lap))
        self.assertLess(renderer.view[0], -20.)
        self.assertGreater(renderer.view[3], 12.)
        self.assertEqual([circle[3] for circle in renderer.circles], list(WHEEL_COLORS) * 2)
        self.assertIn("LapSim · 3 / 3 筆", [item[4] for item in renderer.primitives if item[0] == "text"])

        renderer.view = [-1., 1., -1., 1.]
        renderer.draw()
        self.assertEqual([circle[3] for circle in renderer.circles], list(WHEEL_COLORS))
        outer, inner = renderer.circles
        self.assertEqual(outer[:2], inner[:2])
        self.assertGreater(outer[2], inner[2])
