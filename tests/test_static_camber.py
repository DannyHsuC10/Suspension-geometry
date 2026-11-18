"""Initial camber is a wheel installation, after zero-camber arm construction."""
import copy
import gc
import json
import math
import os
from pathlib import Path
import tempfile
import unittest

import numpy as np

from suspension_geometry_lab import hardpoints
from suspension_geometry_lab.engine import Engine, to_contact_frame
from suspension_geometry_lab.fvsa_search import calculate_lap_peak, search_fvsa
from suspension_geometry_lab.lap_data import LapData, LapPose, calculate_lap_camber
from suspension_geometry_lab.project import default_project, load_project, save_project, validate
from suspension_geometry_lab.project_import import read_import_project
from suspension_geometry_lab.roll_center_search import check_travel
from suspension_geometry_lab.tire_response import tire_orientation
from suspension_geometry_lab.wheel_alignment import installation_matrix
from suspension_geometry_lab.wheel_end_geometry import rotation_matrix


class StaticCamberTests(unittest.TestCase):
    def project(self):
        project = default_project()
        project["front"]["static_camber_deg"] = -2.
        project["rear"]["static_camber_deg"] = 1.5
        return project

    def test_post_installation_preserves_all_hardpoints_centers_and_construction(self):
        base, tilted = Engine(default_project()), Engine(self.project())
        self.assertEqual(hardpoints.from_scene(base.scene), hardpoints.from_scene(tilted.scene))
        self.assertEqual(base.scene.geometry["mount_position_checks"], tilted.scene.geometry["mount_position_checks"])
        for corner in ("FL", "FR", "RL", "RR"):
            before = base.scene.geometry["wheel_projection_geometry"][corner]
            after = tilted.scene.geometry["wheel_projection_geometry"][corner]
            np.testing.assert_array_equal(before["center"], after["center"])
            self.assertFalse(np.array_equal(before["tire"]["vertices"], after["tire"]["vertices"]))
            np.testing.assert_array_equal(before["tire"]["vertices"], after["zero_camber_tire"]["vertices"])
            for name in ("UBJ", "LBJ", "ground", "axis_top"):
                np.testing.assert_array_equal(base.scene.geometry["kingpins"][corner][name], tilted.scene.geometry["kingpins"][corner][name])
            for name, circle in base.scene.geometry["interference_geometry"][corner]["circles"].items():
                np.testing.assert_array_equal(circle["points"], tilted.scene.geometry["interference_geometry"][corner]["circles"][name]["points"])

    def test_static_front_view_is_mirrored_and_contact_follows_loaded_circle(self):
        engine = Engine(self.project())
        for axle, gamma in (("front", -2.), ("rear", 1.5)):
            solver = engine.fv[axle]
            state = solver.solve(0., 0.)
            for key in ("camber_left_deg", "camber_right_deg", "static_camber_deg"):
                self.assertAlmostEqual(state["metrics"][key], gamma, places=9)
            left, right = solver.side(0, True), solver.side(0)
            np.testing.assert_allclose(left["contact"], right["contact"]*[-1, 1], atol=1e-13)
            radius = solver.loaded_radius
            expected = solver.zero["wheel_center"]+radius*np.array([-math.sin(math.radians(gamma)), -math.cos(math.radians(gamma))])
            np.testing.assert_allclose(right["contact"], expected, atol=1e-13)
            for wheel in ("L", "R"):
                self.assertAlmostEqual(to_contact_frame(state["points"][wheel+"_gnd"], state["frame"])[1], 0., places=12)
            np.testing.assert_allclose(state["points"]["RC"], state["points"]["RC0"], atol=1e-12)

    def test_mechanism_travel_is_unchanged_but_wheel_angle_and_contact_are_corrected(self):
        base, tilted = Engine(default_project()), Engine(self.project())
        for axle in ("front", "rear"):
            for travel in (-16., 0., 21.):
                first, last = base.fv[axle].side(travel), tilted.fv[axle].side(travel)
                for key in ("kua", "lak", "IC", "gnd", "wheel_center", "cpb_ru", "cpb_rl"):
                    np.testing.assert_array_equal(first[key], last[key])
                self.assertAlmostEqual(last["wheel_angle"], last["angle"]-math.radians(tilted.project[axle]["static_camber_deg"]))
                normal = np.array([math.cos(last["wheel_angle"]), math.sin(last["wheel_angle"])])
                offset = last["contact"]-last["wheel_center"]
                self.assertAlmostEqual(float(offset@normal), 0., places=12)
                self.assertAlmostEqual(float(np.linalg.norm(offset)), tilted.fv[axle].loaded_radius, places=12)
        solver = tilted.fv["front"]
        state = solver.solve(11., -7.)
        for key, left_side in (("left", True), ("right", False)):
            pairs = ((10.99, -7.), (11.01, -7.)) if left_side else ((11., -7.01), (11., -6.99))
            expected = (solver.solve(*pairs[1], gains=False)["metrics"][f"camber_{key}_deg"]-
                        solver.solve(*pairs[0], gains=False)["metrics"][f"camber_{key}_deg"])/.02
            self.assertAlmostEqual(state["metrics"][f"camber_gain_{key}_deg_per_mm"], expected, places=8)

    def test_lap_before_after_and_steered_camber_use_rotated_axes_with_pitch(self):
        engine = Engine(self.project())
        lap = LapData(((-12., 16., -8., 10.), (16., -12., 10., -8.)),
                      (LapPose(0., 3., 17., .1), LapPose(0., -3., -9., -.1)),
                      "curvature_1_m", ((16., 8.), (-8., -16.)))
        rows = calculate_lap_camber(engine, "front", lap, include_steering=True)["rows"]
        for index, (travels, pose, row) in enumerate(zip(lap.travels, lap.poses, rows)):
            body = rotation_matrix([0, 1, 0], math.radians(-pose.pitch_deg))@rotation_matrix([1, 0, 0], math.radians(pose.roll_deg))
            left, right, _ = engine.fv["front"].wheel_center_sides(*travels[:2])
            for role in ("inner", "outer"):
                corner = row[role+"_wheel"]
                is_left = corner.endswith("L")
                state, side = (left, 1) if is_left else (right, -1)
                installation = installation_matrix(-2., corner)
                suspension = rotation_matrix([1, 0, 0], -state["angle"])
                outward = np.array([0., side, 0.])
                initial = installation@outward
                moved = suspension@initial
                pin = engine.scene.geometry["kingpins"][corner]
                steer = lap.front_steer_deg[index][0 if is_left else 1]
                steered = rotation_matrix(suspension@(pin["UBJ"]-pin["LBJ"]), math.radians(steer))@moved
                for name, vector in (("before", initial), ("after", moved), ("steered", steered)):
                    expected = -math.degrees(math.asin(float((body@vector)[2])))
                    self.assertAlmostEqual(row[f"{role}_camber_{name}_deg"], expected, places=8)
        self.assertEqual(rows[0]["static_camber_deg"], -2.)
        for axle in ("front", "rear"):
            peak = calculate_lap_peak(engine, axle, lap)
            distribution = calculate_lap_camber(engine, axle, lap)["rows"]
            expected = max(abs(r[f"{role}_camber_after_deg"]) for r in distribution for role in ("inner", "outer"))
            self.assertAlmostEqual(peak.peak_deg, expected, places=8)

    def test_wheel_end_tire_model_and_mirrored_contact_share_initial_camber(self):
        engine = Engine(self.project())
        for prefix, axle in (("F", "front"), ("R", "rear")):
            gamma = engine.project[axle]["static_camber_deg"]
            contacts = []
            for suffix in ("L", "R"):
                pose, _, _, _, camber = tire_orientation(engine, prefix+suffix, 0.)
                self.assertAlmostEqual(camber, gamma, places=9)
                contacts.append(pose["contact"])
                offset = pose["contact"]-pose["center"]
                self.assertAlmostEqual(float(offset@pose["normal"]), 0., places=12)
                self.assertAlmostEqual(float(np.linalg.norm(offset)), pose["radius"], places=12)
            np.testing.assert_allclose(contacts[0], contacts[1]*[1, -1, 1], atol=1e-12)
            radii = [engine.solve_we(steer, axle)["metrics"]["material_radius_mm"] for steer in (-25., 0., 25.)]
            np.testing.assert_allclose(radii, radii[0], atol=1e-9)

    def test_searches_use_corrected_geometry_and_camber(self):
        project = self.project()
        engine = Engine(project)
        travels = np.array([-20., -5., 0., 11., 24.])
        check = check_travel(engine.fv["front"], travels)
        reference = [engine.fv["front"].solve(l, r, gains=False)["metrics"] for l in travels for r in travels]
        self.assertAlmostEqual(check.min_height_mm, min(r["rc_y_prime_mm"] for r in reference), places=9)
        lap = LapData(((-12., 16., -8., 10.), (16., -12., 10., -8.)),
                      (LapPose(0., 3., 2., .1), LapPose(0., -3., -1., -.1)), "ay_g")
        # The same lap/limit passes at the zero-camber start but cannot pass
        # with a sufficiently negative installation angle.
        project["front"]["static_camber_deg"] = -5.
        result = search_fvsa(project, "front", lap, 2.)
        self.assertIsNone(result.project)
        self.assertEqual(result.stop_reason, "camber_limit")
        self.assertGreater(result.stopped_peak.peak_deg, 2.)

    def test_optional_legacy_input_and_project_roundtrip_preserve_camber(self):
        source = default_project()
        for axle in ("front", "rear"):
            source[axle].pop("static_camber_deg")
        before = copy.deepcopy(source)
        normalized = validate(source)
        self.assertEqual(source, before)
        self.assertEqual(normalized["front"]["static_camber_deg"], 0.)
        directory = Path(__file__).resolve().parents[1]/".test-tmp"
        directory.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=directory) as temporary:
            path = Path(temporary)/"project.json"
            path.write_text(json.dumps(source), encoding="utf-8")
            imported, missing = read_import_project(path)
            self.assertFalse(missing)
            self.assertEqual(imported["rear"]["static_camber_deg"], 0.)
            save_project(path, self.project())
            self.assertEqual(load_project(path), self.project())
        for value in (float("nan"), -20.1, 20.1):
            bad = self.project()
            bad["front"]["static_camber_deg"] = value
            with self.assertRaises(ValueError):
                validate(bad)


@unittest.skipUnless(os.environ.get("SUSPENSION_GEOMETRY_LAB_GUI_TEST") == "1", "Opt-in native window tests")
class StaticCamberGuiTests(unittest.TestCase):
    def setUp(self):
        from suspension_geometry_lab.gui import App
        project = default_project()
        project["analysis"]["samples"] = 5
        self.app = App(project)
        self.app.update()

    def tearDown(self):
        self.app.dirty = False
        self.app.close()
        self.app.worker_thread.join(timeout=5)
        self.app = None
        gc.collect()

    def test_front_and_rear_have_separate_editable_fields_and_apply_to_all_pages(self):
        for axle, gamma in (("front", -2.), ("rear", 1.5)):
            panel = self.app.panels[axle]
            field = (axle, "static_camber_deg")
            self.assertIn(field, panel.entries)
            self.assertIn("級距 0.1 deg", panel.entry_widgets[field].range_label.cget("text"))
            panel.entries[field].set(str(gamma))
            panel.apply()
            self.assertEqual(self.app.project[axle]["static_camber_deg"], gamma)
            self.assertAlmostEqual(self.app.engine.fv[axle].solve(0, 0)["metrics"]["camber_left_deg"], gamma)
            self.assertEqual(self.app.panels["we"].entries[field].get(), str(gamma).rstrip("0").rstrip("."))

    def test_hardpoint_mode_still_allows_wheel_only_camber_adjustment(self):
        project = copy.deepcopy(self.app.project)
        points = hardpoints.from_scene(self.app.engine.scene)
        project["hardpoints"] = {"enabled": True, "corners": points}
        self.app.apply_project(project)
        panel = self.app.panels["front"]
        field = ("front", "static_camber_deg")
        self.assertEqual(str(panel.entry_widgets[field].cget("state")), "normal")
        panel.entries[field].set("-2.04")
        panel.apply()
        self.assertEqual(self.app.project["front"]["static_camber_deg"], -2.)
        self.assertEqual(self.app.project["hardpoints"]["corners"], points)
        self.assertAlmostEqual(self.app.engine.fv["front"].solve(0, 0)["metrics"]["camber_right_deg"], -2.)


if __name__ == "__main__":
    unittest.main()
