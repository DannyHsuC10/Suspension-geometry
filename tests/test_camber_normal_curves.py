"""Normal overlays use raw camber samples and the histogram's count scale."""
from io import BytesIO
import gc
import math
import os
from pathlib import Path
import time
import unittest
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import numpy as np

from suspension_geometry_lab.analysis import (
    LAP_CAMBER_BIN_DEG, LAP_CAMBER_PLOT, LAP_CAMBER_SERIES_BY_PAGE,
    make_plot, normal_camber_curve,
)
from suspension_geometry_lab.charts import LabChart


def chart_for(spec):
    chart = object.__new__(LabChart)
    chart.canvas = Mock()
    chart.canvas.winfo_width.return_value = 1000
    chart.canvas.winfo_height.return_value = 600
    chart.canvas.winfo_ismapped.return_value = True
    chart.spec = chart.view = chart._plot_bounds = None
    chart.primitives, chart.polygons, chart.circles = [], [], []
    chart.azimuth, chart.elevation = .65, .6
    chart.set(spec)
    return chart


def plot_for(page):
    keys = [entry[0] for entry in LAP_CAMBER_SERIES_BY_PAGE[page]]
    rows = [{key: (sample-2)*.13+group*.24 for group, key in enumerate(keys)} for sample in range(5)]
    return make_plot(page, LAP_CAMBER_PLOT[0], mapping={"rows": rows, "valid": 5, "total": 5})


class CamberNormalCurveTests(unittest.TestCase):
    def test_mean_population_sigma_peak_and_integral_use_raw_angles(self):
        values = np.array([-.11, .03, .11, .18, .37])
        curve = normal_camber_curve(values, "group", "#38bdf8", False)
        mean, sigma = float(np.mean(values)), float(np.std(values, ddof=0))
        self.assertEqual(curve["count"], len(values))
        self.assertAlmostEqual(curve["mean_deg"], mean)
        self.assertAlmostEqual(curve["std_deg"], sigma)
        center = curve["x"].index(mean)
        self.assertAlmostEqual(curve["y"][center], len(values)*LAP_CAMBER_BIN_DEG/(sigma*math.sqrt(2*math.pi)))
        area = sum((b-a)*(lo+hi)/2 for a, b, lo, hi in
                   zip(curve["x"], curve["x"][1:], curve["y"], curve["y"][1:]))
        self.assertAlmostEqual(area/LAP_CAMBER_BIN_DEG, len(values), delta=.001)
        # These distinct angles all fit in a small number of bins; fitting
        # the bin centers would not reproduce this mean and sigma.
        self.assertNotAlmostEqual(mean, .125)

    def test_single_constant_and_empty_samples_do_not_create_invalid_curves(self):
        for samples in ([], [2.], [2., 2., 2.]):
            with self.subTest(samples=samples):
                curve = normal_camber_curve(samples, "group", "#fbbf24", True)
                self.assertEqual((curve["x"], curve["y"]), ([], []))
                self.assertTrue(curve["reason"])
        curve = normal_camber_curve([1., 2., float("nan")], "group", "#fbbf24", True)
        self.assertEqual(curve["count"], 2)
        self.assertTrue(all(math.isfinite(value) for value in curve["x"]+curve["y"]))

    def test_front_rear_and_steering_keep_bar_counts_and_matching_curve_groups(self):
        for page, count in (("front", 4), ("rear", 4), ("we", 6)):
            with self.subTest(page=page):
                spec = plot_for(page)
                self.assertEqual(len(spec["series"]), count)
                self.assertEqual(len(spec["normal_curves"]), count)
                for bar, curve in zip(spec["series"], spec["normal_curves"]):
                    self.assertEqual((bar["name"], bar["color"]), (curve["name"], curve["color"]))
                    self.assertEqual(sum(bar["y"]), curve["count"])
                    self.assertEqual(bool(curve["dash"]), bar["hollow"])
                chart = chart_for(spec)
                self.assertLessEqual(chart.view[0], spec["normal_x_bounds"][0])
                self.assertGreaterEqual(chart.view[1], spec["normal_x_bounds"][1])
                self.assertGreaterEqual(chart.view[3], max(max(c["y"]) for c in spec["normal_curves"]))

    def test_curves_export_as_smooth_lines_and_follow_visibility_without_refitting(self):
        from PIL import Image
        spec = plot_for("we")
        chart = chart_for(spec)
        ns = "{http://www.w3.org/2000/svg}"

        def curves():
            return [item for item in ET.fromstring(chart.svg()).findall(ns+"polyline")
                    if item.attrib["stroke-width"] == "2.5"]

        self.assertEqual(len(curves()), 6)
        self.assertTrue(any("stroke-dasharray" in item.attrib for item in curves()))
        self.assertTrue(all(len(item.attrib["points"].split()) > 100 for item in curves()))
        view = chart.view.copy()
        chart.set({**spec, "hidden_series": [spec["series"][0]["name"]]}, preserve_view=True)
        self.assertEqual(chart.view, view)
        self.assertEqual(len(curves()), 5)
        chart.set({**spec, "show_normal_curves": False}, preserve_view=True)
        self.assertEqual(chart.view, view)
        self.assertFalse(curves())
        self.assertTrue(any(item[0] == "rect" for item in chart.primitives))
        chart.set({**spec, "hidden_series": [s["name"] for s in spec["series"]]}, preserve_view=True)
        self.assertFalse(curves())
        chart.set(spec)
        output = BytesIO()
        output.name = "normal-curves.png"
        chart.png(output)
        with Image.open(output) as rendered:
            self.assertEqual(rendered.size, (2000, 1200))

    def test_histogram_hover_includes_raw_fit_parameters(self):
        spec = plot_for("front")
        chart = chart_for(spec)
        left, top, right, _ = chart.bounds
        xmin, xmax, _, _ = chart.view
        angle = (spec["bin_edges"][0]+spec["bin_edges"][1])/2
        chart.hover(Mock(x=left+(angle-xmin)/(xmax-xmin)*(right-left), y=top+40))
        label = chart.canvas.create_text.call_args.kwargs["text"]
        self.assertIn("μ=", label)
        self.assertIn("σ=", label)


@unittest.skipUnless(os.environ.get("SUSPENSION_GEOMETRY_LAB_GUI_TEST") == "1", "Opt-in native window tests")
class CamberNormalCurveGuiTests(unittest.TestCase):
    def test_curve_switch_and_category_filters_work_in_all_three_pages(self):
        from suspension_geometry_lab.gui import App
        from suspension_geometry_lab.lap_data import LapData, LapPose
        from suspension_geometry_lab.project import default_project
        project = default_project()
        project["analysis"]["samples"] = 5
        app = App(project)
        app.update()
        try:
            lap = LapData(tuple((float(i), -float(i), float(i)/2, -float(i)/2) for i in (-8, -4, 1, 4, 8)),
                          tuple(LapPose(0., i/4, i/8, 1.) for i in (-8, -4, 1, 4, 8)),
                          "ay_g", tuple((i/2, i/3) for i in (-8, -4, 1, 4, 8)))
            app.set_lap_data(lap, Path("normal-fit.csv"))
            for page in ("front", "rear", "we"):
                panel = app.panels[page]
                app.notebook.select(panel)
                app.update()
                panel.plot.set(LAP_CAMBER_PLOT[1])
                panel.request()
                end = time.monotonic()+20
                while not getattr(panel, "ready_for_export", False) and time.monotonic() < end:
                    app.update()
                    time.sleep(.01)
                self.assertTrue(panel.ready_for_export, app.status.get())
                self.assertTrue(panel.normal_curves_check.winfo_viewable())
                self.assertTrue(panel.chart.spec["show_normal_curves"])
                self.assertTrue(any(p[0] == "line" and p[3] == 2.5 for p in panel.chart.primitives))
                view = panel.chart.view.copy()
                panel.normal_curves_check.invoke()
                self.assertFalse(panel.chart.spec["show_normal_curves"])
                self.assertEqual(panel.chart.view, view)
                self.assertFalse(any(p[0] == "line" and p[3] == 2.5 for p in panel.chart.primitives))
                panel.normal_curves_check.invoke()
                metric, label, _, _ = LAP_CAMBER_SERIES_BY_PAGE[page][0]
                panel.camber_visible[metric].set(False)
                panel.render()
                self.assertIn(label, panel.chart.spec["hidden_series"])
        finally:
            app.dirty = False
            app.close()
            app.worker_thread.join(timeout=5)
            app = None
            gc.collect()


if __name__ == "__main__":
    unittest.main()
