"""Exercise the browser worker boundary and packaged TV dependencies."""
import csv
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "docs"))
import lab_worker as worker
from suspension_geometry_lab import hardpoints
from suspension_geometry_lab.project import default_project, steering_settings
from suspension_geometry_lab.steering import solve_steering
from suspension_geometry_lab.tv_analysis import steering_camber_metrics


class BrowserTVTests(unittest.TestCase):
    def setUp(self):
        self.project = default_project()
        self.project["steering"] = steering_settings(self.project)
        worker.install_project(self.project)
        worker.clear_lap()

    def call(self, action, **payload):
        result = json.loads(worker.dispatch(action, json.dumps(payload)))
        self.assertTrue(result["ok"], result.get("error"))
        return result["data"]

    def test_all_charts_and_current_gains_use_desktop_solver(self):
        summary = self.call("init")
        self.assertIn("tv", summary["views"])
        self.assertEqual(len(summary["views"]["tv"]["plots"]), 6)
        pose = solve_steering(worker._engine, "front", self.project["steering"]["front"], travel_mm=15.)
        expected = steering_camber_metrics(worker._engine, "front", pose)
        for plot in summary["views"]["tv"]["plots"]:
            result = self.call("plot", page="tv", plot=plot["key"], axle="front", values=[15.])
            self.assertIsNone(result["error"])
            self.assertTrue(result["spec"])
            self.assertTrue(result["tvRows"])
            for key, value in expected.items():
                self.assertAlmostEqual(result["state"]["metrics"][key], value)

    def test_rear_enable_and_stops_and_static_step(self):
        result = self.call("plot", page="tv", plot="top_view", axle="rear", values=[20.])
        self.assertEqual(result["controls"], [])
        self.assertEqual(result["state"]["metrics"]["rack_travel_mm"], 0.)
        self.project["steering"]["rear"].update(enabled=True, rack_limit_left_mm=10., rack_limit_right_mm=12.)
        worker.install_project(self.project)
        result = self.call("plot", page="tv", plot="steer", axle="rear", values=[10.1])
        self.assertIsNone(result["state"])
        self.assertIn("outside the allowed range", result["error"])
        self.assertEqual((result["controls"][0]["min"], result["controls"][0]["max"]), (-12., 10.))
        step = self.call("export-data", page="static", plot="iso", values=[], kind="step", stepOptions={"include_tires": False})
        self.assertEqual(step["content"].count(" tie rod'"), 4)

    def test_hardpoint_editor_and_static_csv_use_neutral_tie_rod_endpoints(self):
        corners = hardpoints.from_scene(worker._engine.scene)
        self.project["hardpoints"] = {"enabled": True, "corners": corners}
        metadata = self.call("project-prepare", project=self.project)
        self.assertEqual(set(metadata["optionalPoints"]), {"RACK", "TOE"})
        self.assertNotIn("RACK", metadata["project"]["hardpoints"]["corners"]["FL"])
        self.assertTrue(all({"RACK", "TOE"} <= points.keys() for points in metadata["points"].values()))
        rack_x, _, rack_z = metadata["points"]["FL"]["RACK"]
        toe_x, toe_y, toe_z = metadata["points"]["FL"]["TOE"]
        hardpoints.set_point(corners, "FL", "RACK", [rack_x, .28, rack_z])
        hardpoints.set_point(corners, "FL", "TOE", [toe_x, toe_y, toe_z + .012])

        applied = self.call("project-apply", project=self.project)["project"]
        summary = self.call("project", project=applied)
        self.assertEqual(summary["steering"], self.project["steering"])
        self.assertAlmostEqual(summary["steeringGeometry"]["front"]["rack_length"], .56)
        self.assertEqual(summary["steeringGeometry"]["rear"], {})
        tv = self.call("plot", page="tv", plot="top_view", axle="front", values=[3.])
        self.assertIsNone(tv["error"])
        self.assertAlmostEqual(tv["state"]["metrics"]["tie_rod_z"], toe_z + .012)
        self.assertAlmostEqual(tv["state"]["metrics"]["rack_travel_mm"], 3.)

        csv_text = self.call("export-data", page="static", plot="iso", values=[], kind="points")["content"]
        rows = {row["name"]: row for row in csv.DictReader(io.StringIO(csv_text.lstrip("\ufeff")))}
        for corner in hardpoints.CORNERS:
            for key in ("RACK", "TOE"):
                row = rows[f"{corner}_{key}"]
                self.assertEqual([float(row[f"{axis}_m"]) for axis in "xyz"],
                                 corners[corner].get(key, metadata["points"][corner][key]))

    def test_lap_operating_points_and_csv_report(self):
        csv_text = (ROOT / "projects" / "lap_result.csv").read_text(encoding="utf-8-sig")
        self.call("lap_import", name="lap.csv", text=csv_text)
        for plot in ("steer", "ackermann", "camber", "camber_gain", "steering_camber_gain"):
            result = self.call("plot", page="tv", plot=plot, axle="front", values=[10.], showLap=True)
            self.assertTrue(result["spec"]["lap_points"])
            self.assertIn("estimated", result["tvNote"])
        data = self.call("export-data", page="tv", plot="camber", axle="front", values=[10.], kind="state")
        row, = list(csv.DictReader(io.StringIO(data["content"].lstrip("\ufeff"))))
        self.assertIn("left_camber_gain_deg_per_mm", row)
        sweep = self.call("export-data", page="tv", plot="camber", axle="front", values=[10.], kind="mapping")
        self.assertEqual(len(list(csv.DictReader(io.StringIO(sweep["content"].lstrip("\ufeff"))))), 121)
        report = self.call("export-data", page="tv", plot="camber", axle="front", values=[10.], kind="report", svg="<svg/>")
        self.assertIn("Top View (TV)", report["content"])

    def test_packaged_worker_has_all_tv_sources(self):
        with tempfile.TemporaryDirectory(dir=ROOT / ".test-tmp") as tmp:
            result = subprocess.run([sys.executable, "-B", str(ROOT / "docs" / "build.py"), tmp], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            for source in ("tv_worker.py", "geometry-tv.js", "suspension_geometry_lab/steering.py",
                           "suspension_geometry_lab/steering_chart.py", "suspension_geometry_lab/tv_analysis.py",
                           "suspension_geometry_lab/tv_lap.py"):
                self.assertTrue((Path(tmp) / source).is_file(), source)
            code = "import sys;sys.path.insert(0,'.');import lab_worker;assert 'tv' in lab_worker.handle('init',{})['views']"
            result = subprocess.run([sys.executable, "-I", "-B", "-c", code], cwd=tmp, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
