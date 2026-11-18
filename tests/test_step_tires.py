"""STEP tires are exact paired circles with optional installation camber."""
import copy
import gc
import os
from pathlib import Path
import re
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np

from suspension_geometry_lab import hardpoints
from suspension_geometry_lab.engine import Engine
from suspension_geometry_lab.export import write_step
from suspension_geometry_lab.project import default_project
from suspension_geometry_lab.static.fv.wireframe_geometry import project_fv


def parse_step(data):
    entities = dict(re.findall(r"^(#\d+)=(.+);$", data, re.MULTILINE))
    for value in entities.values():
        if any(ref not in entities for ref in re.findall(r"#\d+", value)):
            raise AssertionError("Dangling STEP reference")
    vectors = {ref: np.array([float(value) for value in match[1].split(",")])
               for ref, value in entities.items()
               if (match := re.fullmatch(r"(?:CARTESIAN_POINT|DIRECTION)\('',\(([^)]+)\)\)", value))}
    circles, polylines = {}, {}
    for value in entities.values():
        circle = re.fullmatch(r"CIRCLE\('([^']+)',(#\d+),([^)]*)\)", value)
        if circle:
            placement = re.fullmatch(r"AXIS2_PLACEMENT_3D\('',(#\d+),(#\d+),(#\d+)\)", entities[circle[2]])
            circles[circle[1]] = {"center": vectors[placement[1]], "normal": vectors[placement[2]],
                                  "reference": vectors[placement[3]], "radius": float(circle[3])}
        line = re.fullmatch(r"POLYLINE\('([^']+)',\(([^)]+)\)\)", value)
        if line:
            polylines[line[1]] = np.array([vectors[ref] for ref in line[2].split(",")])
    return circles, polylines


class StepTireTests(unittest.TestCase):
    def setUp(self):
        self.project = default_project()
        self.project["front"]["static_camber_deg"] = -3.
        self.project["rear"]["static_camber_deg"] = 2.
        self.engine = Engine(self.project)

    def export(self, page, state=None, **options):
        directory = Path(__file__).resolve().parents[1]/".test-tmp"
        directory.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=directory) as temporary:
            path = Path(temporary)/"輪胎.step"
            write_step(path, self.engine, page, state, **options)
            data = path.read_text(encoding="ascii")
        self.assertIn("SI_UNIT(.MILLI.,.METRE.)", data)
        return parse_step(data)

    def assert_pair(self, circles, corner, expected_center, expected_axis=None):
        first, last = (circles[f"{corner} tire side {side}"] for side in ("-", "+"))
        np.testing.assert_allclose((first["center"]+last["center"])/2, np.asarray(expected_center)*1000,
                                   rtol=0, atol=1e-8)
        self.assertAlmostEqual(np.linalg.norm(last["center"]-first["center"]), self.engine.car.tire_width*1000, places=7)
        for circle in (first, last):
            self.assertAlmostEqual(circle["radius"], self.engine.car.free_radius*1000, places=8)
            self.assertAlmostEqual(np.linalg.norm(circle["normal"]), 1., places=8)
            self.assertAlmostEqual(np.linalg.norm(circle["reference"]), 1., places=8)
            self.assertAlmostEqual(float(circle["normal"]@circle["reference"]), 0., places=8)
            if expected_axis is not None:
                axis = np.asarray(expected_axis)/np.linalg.norm(expected_axis)
                self.assertAlmostEqual(abs(float(axis@circle["normal"])), 1., places=8)

    def test_static_full_vehicle_has_eight_exact_circles_and_unchanged_links(self):
        before = copy.deepcopy(self.engine.project)
        circles, links = self.export("static")
        self.assertEqual(len(circles), 8)
        self.assertEqual(len(links), 20)
        for corner, wheel in self.engine.scene.geometry["wheel_projection_geometry"].items():
            self.assert_pair(circles, corner, wheel["center"], wheel["installation_matrix"][:, 1])
        flat, flat_links = self.export("static", include_static_camber=False)
        for corner, wheel in self.engine.scene.geometry["wheel_projection_geometry"].items():
            self.assert_pair(flat, corner, wheel["center"], [0., 1., 0.])
        for name in links:
            np.testing.assert_array_equal(links[name], flat_links[name])
        self.assertEqual(self.engine.project, before)

    def test_current_fv_centers_follow_pose_and_only_static_offset_is_optional(self):
        zero_project = copy.deepcopy(self.project)
        for axle in ("front", "rear"):
            zero_project[axle]["static_camber_deg"] = 0.
        zero_engine = Engine(zero_project)
        for page, prefix in (("front", "F"), ("rear", "R")):
            state = self.engine.fv[page].solve(15., -8.)
            zero = zero_engine.fv[page].solve(15., -8.)
            tilted, _ = self.export(page, state)
            flat, _ = self.export(page, state, include_static_camber=False)
            self.assertEqual(len(tilted), 4)
            wheel_paths = [line["points"] for line in state["lines"] if line["name"] == "wheel"]
            zero_paths = [line["points"] for line in zero["lines"] if line["name"] == "wheel"]
            x = self.engine.scene.geometry[page].longitudinal_x
            for suffix, outline, zero_outline in zip(("L", "R"), wheel_paths, zero_paths):
                center = project_fv(state["points"][suffix+"_wheel_center"], x)
                width_vector = project_fv(np.asarray(outline)[1]-outline[0], 0.)
                zero_vector = project_fv(np.asarray(zero_outline)[1]-zero_outline[0], 0.)
                self.assert_pair(tilted, prefix+suffix, center, width_vector)
                self.assert_pair(flat, prefix+suffix, center, zero_vector)
                # Unchecked still follows suspension-induced camber.
                self.assertGreater(abs(flat[prefix+suffix+" tire side +"]["normal"][2]), 1e-4)

    def test_sv_uses_current_axle_centers_and_actual_track_positions(self):
        state = self.engine.solve_sv(13., -6.)
        circles, _ = self.export("sv", state)
        self.assertEqual(len(circles), 8)
        for axle, prefix, track in (("front", "F", self.engine.car.tf), ("rear", "R", self.engine.car.tr)):
            center = self.engine.scene.geometry["side_view"].project(state["points"][axle+"_center"])
            for suffix, side in (("L", 1), ("R", -1)):
                expected = center.copy()
                expected[1] = side*track/2
                self.assert_pair(circles, prefix+suffix, expected)

    def test_hardpoint_mode_keeps_same_tire_installation_and_original_xyz(self):
        points = hardpoints.from_scene(self.engine.scene)
        self.project["hardpoints"] = {"enabled": True, "corners": points}
        self.engine = Engine(self.project)
        circles, _ = self.export("static")
        self.assertEqual(len(circles), 8)
        for corner, wheel in self.engine.scene.geometry["wheel_projection_geometry"].items():
            self.assert_pair(circles, corner, wheel["center"], wheel["installation_matrix"][:, 1])
        self.assertEqual(hardpoints.from_scene(self.engine.scene), points)

    def test_tires_can_be_omitted_and_invalid_tires_cannot_overwrite_a_file(self):
        circles, links = self.export("front", self.engine.fv["front"].solve(0, 0), include_tires=False)
        self.assertFalse(circles)
        self.assertNotIn("wheel", links)
        self.engine.car.free_radius = float("nan")
        directory = Path(__file__).resolve().parents[1]/".test-tmp"
        with tempfile.TemporaryDirectory(dir=directory) as temporary:
            path = Path(temporary)/"keep.step"
            path.write_bytes(b"original")
            with self.assertRaises(ValueError):
                write_step(path, self.engine, "static")
            self.assertEqual(path.read_bytes(), b"original")


@unittest.skipUnless(os.environ.get("SUSPENSION_GEOMETRY_LAB_GUI_TEST") == "1", "Opt-in native window tests")
class StepTireGuiTests(unittest.TestCase):
    def setUp(self):
        from suspension_geometry_lab.gui import App
        project = default_project()
        project["front"]["static_camber_deg"] = -3.
        project["analysis"]["samples"] = 5
        self.app = App(project)
        self.app.update()
        self.panel = self.app.panels["static"]
        self.app.notebook.select(self.panel)
        self.app.update()
        end = time.monotonic()+20
        while not self.panel.ready_for_export and time.monotonic() < end:
            self.app.update()
            time.sleep(.01)
        self.assertTrue(self.panel.ready_for_export)

    def tearDown(self):
        self.app.dirty = False
        self.app.close()
        self.app.worker_thread.join(timeout=5)
        self.panel = None
        self.app = None
        gc.collect()

    def test_options_dialog_controls_camber_and_remembers_selection(self):
        directory = Path(__file__).resolve().parents[1]/".test-tmp"
        with tempfile.TemporaryDirectory(dir=directory) as temporary:
            path = Path(temporary)/"tires.step"

            def choose(dialog):
                self.assertTrue(dialog.include_tires.get())
                self.assertTrue(dialog.include_static_camber.get())
                dialog.include_static_camber.set(False)
                dialog.accept()

            with patch.object(self.app, "wait_window", side_effect=choose), \
                    patch("suspension_geometry_lab.gui.filedialog.asksaveasfilename", return_value=str(path)):
                self.app.export("step")
            circles, _ = parse_step(path.read_text(encoding="ascii"))
            self.assertEqual(len(circles), 8)
            self.assertFalse(self.app.step_options["include_static_camber"])
            for circle in circles.values():
                np.testing.assert_allclose(circle["normal"], [0., 1., 0.], atol=1e-10)

            def choose_no_tires(dialog):
                self.assertFalse(dialog.include_static_camber.get())
                dialog.include_tires.set(False)
                dialog.refresh()
                self.assertTrue(dialog.camber_check.instate(["disabled"]))
                dialog.accept()

            with patch.object(self.app, "wait_window", side_effect=choose_no_tires), \
                    patch("suspension_geometry_lab.gui.filedialog.asksaveasfilename", return_value=str(path)):
                self.app.export("step")
            self.assertFalse(parse_step(path.read_text(encoding="ascii"))[0])

    def test_cancelled_options_do_not_open_destination_or_export(self):
        with patch.object(self.app, "wait_window", side_effect=lambda dialog: dialog.destroy()), \
                patch("suspension_geometry_lab.gui.filedialog.asksaveasfilename") as save, \
                patch("suspension_geometry_lab.gui.write_step") as write:
            self.app.export("step")
            save.assert_not_called()
            write.assert_not_called()


if __name__ == "__main__":
    unittest.main()
