"""Contract checks for shared static provenance, constraints and exported projects."""
import copy
import csv
import gc
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from suspension_geometry_lab.project import default_project, load_project, save_project, validate, Car, SuspensionF, SuspensionR, DERIVED_AXLE_FIELDS
from suspension_geometry_lab.engine import Engine, Unreachable, contact_frame, to_contact_frame, rotate2
from suspension_geometry_lab.analysis import ANTI_RESULT_LABELS, RESULT_LABELS, calculate_mapping, make_plot, static_point_rows, PLOTS
from suspension_geometry_lab.export import point_rows, write_report
from suspension_geometry_lab.static.fv.wireframe_geometry import build_wireframe_scene

TEST_TMP = Path(__file__).resolve().parents[1] / ".test-tmp"
TEST_TMP.mkdir(exist_ok=True)


class ProjectTests(unittest.TestCase):
    def test_staged_app_runs_without_vehicle_lab_or_legacy_imports(self):
        source = '''
import importlib.abc
from pathlib import Path
import runpy
import sys

root = Path.cwd()
assert not (root / 'new').exists()
assert not (root / 'vehicle_lab').exists()
sys.path.insert(0, str(root))

class NoLegacy(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('vehicle_lab', 'static', 'suspension_geo', 'new'):
            raise AssertionError('Legacy import: ' + fullname)
sys.meta_path.insert(0, NoLegacy())
search_path = sys.path.copy()

from suspension_geometry_lab.project import default_project, load_project, save_project
from suspension_geometry_lab.engine import Engine
from suspension_geometry_lab.analysis import calculate_mapping, make_plot
from suspension_geometry_lab.gui import App
from suspension_geometry_lab.export import write_report
from suspension_geometry_lab.tire_response import DEFAULT_TIR, TIR_DIR, calculate_tire_response, load_tire_parameters
from suspension_geometry_lab.static.fv.tk_wireframe import WireframeViewer

project = default_project()
project['analysis']['samples'] = 5
save_project(root / 'project.json', project)
engine = Engine(load_project(root / 'project.json'))
states = {'front': engine.fv['front'].solve(-10, 20),
          'rear': engine.fv['rear'].solve(15, -5),
          'sv': engine.solve_sv(-10, 20), 'we': engine.solve_we(20)}
for page, state in states.items():
    assert state['metrics']
    assert state['points']
    assert make_plot(page, 'motion', state, engine=engine)
    assert calculate_mapping(engine, page)['rows']
assert make_plot('static', 'iso', engine=engine)['world_lines']
tire_rows, _ = calculate_tire_response(engine, load_tire_parameters(TIR_DIR / DEFAULT_TIR),
                                       corner='FR', steer_deg=10, slip_angles_deg=[3], slip_ratio=.03)
assert abs(tire_rows[0]['mz_nm']) > 0
assert sys.path == search_path
for name, module in tuple(sys.modules.items()):
    if name.split('.')[0] == 'suspension_geometry_lab':
        assert Path(module.__file__).resolve().is_relative_to(root), module.__file__

sys.argv = ['suspension_geometry_lab', '--help']
try:
    runpy.run_module('suspension_geometry_lab', run_name='__main__')
except SystemExit as error:
    assert error.code == 0
else:
    raise AssertionError('CLI did not exit after --help')
'''
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
            staged = Path(tmp)
            for package in ("suspension_geometry_lab",):
                shutil.copytree(root / package, staged / package,
                                ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
            result = subprocess.run([sys.executable, "-I", "-B", "-c", source], cwd=staged,
                                    capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_roundtrip_and_reject_nonfinite_unknown_and_fractional_grid(self):
        project = default_project()
        project["front"]["fvsa"] = .923
        with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
            path = Path(tmp) / "車輛.json"
            save_project(path, project)
            self.assertEqual(load_project(path), project)
            original = path.read_bytes()
            for group, key, value in (("front", "kpi_deg", math.nan), ("vehicle", "mass", True),
                                      ("analysis", "samples", 5.2), ("analysis", "droop_mm", -1)):
                bad = copy.deepcopy(project)
                bad[group][key] = value
                with self.assertRaises(ValueError):
                    save_project(path, bad)
                self.assertEqual(path.read_bytes(), original)
            project["front"]["unknown"] = 1
            with self.assertRaises(ValueError):
                validate(project)

    def test_engine_needs_no_legacy_design_defaults_or_plotting_scipy(self):
        with patch.object(Car, "__init__", side_effect=AssertionError("legacy defaults")), \
             patch.object(SuspensionF, "__init__", side_effect=AssertionError("legacy defaults")), \
             patch.object(SuspensionR, "__init__", side_effect=AssertionError("legacy defaults")):
            Engine(default_project())
        source = '''
import importlib.abc, sys
class NoPlotting(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('matplotlib', 'mpl_toolkits', 'scipy'):
            raise AssertionError(fullname)
sys.meta_path.insert(0, NoPlotting())
from suspension_geometry_lab.project import default_project
from suspension_geometry_lab.engine import Engine
e = Engine(default_project())
assert abs(e.fv['front'].solve(0,0)['metrics']['rc_y_prime_mm']-35)<1e-7
'''
        result = subprocess.run([sys.executable, "-B", "-c", source], capture_output=True, text=True,
                                cwd=Path(__file__).resolve().parents[1])
        self.assertEqual(result.returncode, 0, result.stderr)


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.engine = Engine(default_project())

    def test_lap_camber_compares_the_same_cornering_samples_in_the_road_frame(self):
        from suspension_geometry_lab.lap_data import LapData, LapPose, calculate_lap_camber
        lap = LapData(
            ((-12, 16, -8, 10), (16, -12, 10, -8), (-12, 16, -8, 10),
             (-12, 16, -8, 10), (0, 0, 0, 0), (100, 0, 100, 0)),
            (LapPose(0, 3, 0, .1), LapPose(0, -3, 0, -.1), LapPose(9, 3, 0, .1),
             LapPose(0, 3, 10, .1), LapPose(0, 2, 0, 0), LapPose(0, 3, 0, .1)),
            "curvature_1_m",
        )
        for axle, prefix in (("front", "F"), ("rear", "R")):
            result = calculate_lap_camber(self.engine, axle, lap)
            self.assertEqual((result["valid"], result["total"], result["straight"]), (4, 5, 1))
            left, right, heaved, pitched = result["rows"]
            for role, expected_before, left_wheel, right_wheel in (
                    ("inner", -3, "L", "R"), ("outer", 3, "R", "L")):
                self.assertEqual(left[f"{role}_wheel"], prefix + left_wheel)
                self.assertEqual(right[f"{role}_wheel"], prefix + right_wheel)
                before, after = f"{role}_camber_before_deg", f"{role}_camber_after_deg"
                self.assertAlmostEqual(left[before], expected_before)
                self.assertAlmostEqual(left[after]-left[before], left[f"{role}_camber_body_deg"])
                self.assertAlmostEqual(left[after], right[after])
                self.assertAlmostEqual(left[after], heaved[after])
                self.assertNotAlmostEqual(left[after], pitched[after], places=4)
                self.assertAlmostEqual(left[f"{role}_center_road_z_mm"]-heaved[f"{role}_center_road_z_mm"], 9)

    def test_lap_front_steering_rotates_the_suspension_pose_and_preserves_wheel_roles(self):
        from suspension_geometry_lab.lap_data import LapData, LapPose, calculate_lap_camber
        from suspension_geometry_lab.wheel_end_geometry import rotation_matrix
        lap = LapData(
            ((0, 0, 0, 0), (-12, 16, 0, 0), (16, -12, 0, 0), (-12, 16, 0, 0)),
            (LapPose(0, 0, 0, .1), LapPose(0, 3, 1, .1), LapPose(0, -3, 1, -.1), LapPose(0, 3, 1, .1)),
            "curvature_1_m", ((12, 9), (16, 8), (-8, -16), (0, 0)),
        )
        rows = calculate_lap_camber(self.engine, "front", lap, include_steering=True)["rows"]
        fv = calculate_lap_camber(self.engine, "front", lap)["rows"]
        self.assertEqual(len(rows), len(fv))
        for baseline, steered in zip(fv, rows):
            self.assertEqual(baseline, {key: steered[key] for key in baseline})
        self.assertAlmostEqual(rows[0]["outer_camber_steered_deg"],
                               self.engine.solve_we(9)["metrics"]["wheel_tilt_deg"])
        for role in ("inner", "outer"):
            self.assertAlmostEqual(rows[1][f"{role}_camber_steered_deg"], rows[2][f"{role}_camber_steered_deg"])
            self.assertAlmostEqual(rows[3][f"{role}_camber_steered_deg"], rows[3][f"{role}_camber_after_deg"])
        self.assertEqual((rows[2]["inner_wheel"], rows[2]["inner_steer_deg"]), ("FR", -16))
        self.assertEqual((rows[2]["outer_wheel"], rows[2]["outer_steer_deg"]), ("FL", -8))

        _, right, _ = self.engine.fv["front"].wheel_center_sides(-12, 16)
        pin = self.engine.scene.geometry["kingpins"]["FR"]
        static_axis = pin["UBJ"]-pin["LBJ"]
        suspension = rotation_matrix([1, 0, 0], -right["angle"])
        body = rotation_matrix([0, 1, 0], math.radians(-1)) @ rotation_matrix([1, 0, 0], math.radians(3))
        wheel_axis = suspension @ [0, -1, 0]
        steered = rotation_matrix(suspension @ static_axis, math.radians(8)) @ wheel_axis
        expected = -math.degrees(math.asin((body @ steered)[2]))
        self.assertAlmostEqual(rows[1]["outer_camber_steered_deg"], expected)

    def test_lap_wheel_center_inputs_reproduce_contact_travel_geometry(self):
        for axle in ("front", "rear"):
            solver = self.engine.fv[axle]
            pose = solver.solve(16, -10, wheel_center=True)
            centers = [pose["points"][side+"_wheel_center"][1] for side in ("L", "R")]
            np.testing.assert_allclose((np.array(centers)-solver.zero["wheel_center"][1])*1000,
                                       [16, -10], atol=1e-7)
            m = pose["metrics"]
            self.assertGreater(m["travel_left_mm"], 16)
            contact = solver.solve(m["travel_left_mm"], m["travel_right_mm"])["metrics"]
            for metric in ("camber_left_deg", "camber_gain_right_deg_per_mm", "rc_y_prime_mm"):
                self.assertAlmostEqual(m[metric], contact[metric], places=6)
        pose = self.engine.solve_sv(12, -8, wheel_center=True)
        for axle, travel in (("front", 12), ("rear", -8)):
            center_z = self.engine.sv[axle].template.wheel_center[1]
            self.assertAlmostEqual((pose["points"][axle+"_center"][1]-center_z)*1000, travel, places=7)
        m = pose["metrics"]
        contact = self.engine.solve_sv(m["travel_front_mm"], m["travel_rear_mm"])["metrics"]
        self.assertAlmostEqual(m["anti_brake_front_pct"], contact["anti_brake_front_pct"], places=6)

    def test_default_scene_matches_authoritative_static_scene_exactly(self):
        from suspension_geometry_lab.project import static_objects
        baseline = build_wireframe_scene(*static_objects(default_project()))
        self.assertEqual(len(baseline.lines), len(self.engine.scene.lines))
        for a, b in zip(baseline.lines, self.engine.scene.lines):
            np.testing.assert_allclose(a.points, b.points, atol=1e-12)
            self.assertEqual((a.color, a.group, a.dash), (b.color, b.group, b.dash))

    def test_fv_lengths_actual_travel_symmetry_and_reentrant_state(self):
        for axle, height in (("front", 35), ("rear", 40)):
            solver = self.engine.fv[axle]
            z = solver.zero
            zero = solver.solve(0, 0)
            self.assertAlmostEqual(zero["metrics"]["rc_y_prime_mm"], height, places=7)
            for travel in (-25, -10, 0, 10, 25):
                s = solver.side(travel)
                self.assertAlmostEqual(s["gnd"][1]*1000, travel, places=6)
                for a, b in (("cpb_ru", "kua"), ("cpb_rl", "lak"), ("kua", "lak"), ("wheel_center", "gnd")):
                    self.assertAlmostEqual(np.linalg.norm(s[a]-s[b]), np.linalg.norm(z[a]-z[b]), places=10)
                both = solver.solve(travel, travel)["metrics"]
                self.assertAlmostEqual(both["rc_x_prime_mm"], 0, places=6)
                self.assertAlmostEqual(both["camber_left_deg"], both["camber_right_deg"], places=8)
            a, b = solver.solve(-15, 20)["metrics"], solver.solve(20, -15)["metrics"]
            self.assertAlmostEqual(a["roll_deg"], -b["roll_deg"], places=7)
            self.assertAlmostEqual(a["rc_x_prime_mm"], -b["rc_x_prime_mm"], places=7)
            self.assertEqual(solver.solve(0, 0), zero)
            with self.assertRaises(Unreachable):
                solver.solve(1000, 0)

    def test_fv_pose_and_gain_partial_derivative(self):
        solver = self.engine.fv["front"]
        state = solver.pose(5, 1.2)["metrics"]
        self.assertAlmostEqual(state["heave_mm"], 5, places=7)
        self.assertAlmostEqual(state["roll_deg"], 1.2, places=6)
        s = solver.solve(10, -5)["metrics"]
        lo = solver.solve(9.9, -5, False)["metrics"]
        hi = solver.solve(10.1, -5, False)["metrics"]
        self.assertAlmostEqual(s["camber_gain_left_deg_per_mm"], (hi["camber_left_deg"]-lo["camber_left_deg"])/.2, places=6)

    def test_sv_tangent_constraints_rigidity_and_pose_roundtrip(self):
        baseline = self.engine.solve_sv(0, 0)
        for axle in ("front", "rear"):
            motion = self.engine.sv[axle]
            initial = motion.template.initial_ic.copy()
            for travel in (-25, 0, 25):
                s = motion.side(travel)
                self.assertAlmostEqual(s["contact"][1]*1000, travel, places=6)
                self.assertAlmostEqual(np.linalg.norm(s["au"]-s["al"]), motion.length, places=10)
                for k, p, d in (("au", motion.au, motion.directions[0]), ("al", motion.al, motion.directions[1])):
                    displacement = s[k]-p
                    self.assertAlmostEqual(np.dot(displacement, [-d[1], d[0]]), 0, places=10)
            np.testing.assert_array_equal(motion.template.initial_ic, initial)
        pose = self.engine.sv_pose(4, -.6)["metrics"]
        self.assertAlmostEqual(pose["heave_mm"], 4, places=7)
        self.assertAlmostEqual(pose["pitch_deg"], -.6, places=6)
        self.assertEqual(self.engine.solve_sv(0, 0), baseline)

    def test_svsa_and_axle_anti_independently_set_static_ic(self):
        project = default_project()
        e = Engine(project)
        static = e.scene.geometry["side_view"].front
        distance = abs(static.initial_ic[0]-static.contact_point[0])
        state = e.solve_sv(0, 0)["metrics"]
        self.assertAlmostEqual(state["anti_brake_front_pct"], 20)
        self.assertAlmostEqual(state["svsa_front_mm"], distance*1000)
        self.assertAlmostEqual(e.derived_parameters["front"]["svsa"], distance)
        project["front"]["svsa"] = 2.5
        changed = Engine(project)
        self.assertNotAlmostEqual(changed.derived_parameters["front"]["svsa"], distance)
        self.assertAlmostEqual(changed.solve_sv(0, 0)["metrics"]["anti_brake_front_pct"], 20)
        self.assertAlmostEqual(changed.solve_sv(0, 0)["metrics"]["svsa_front_mm"], 2500)

    def test_sv_static_trail_matches_signed_parameter_and_hardpoint_designs(self):
        from suspension_geometry_lab import hardpoints
        project = default_project()
        project["front"].update(mechanical_trail=.023, caster_deg=13.)
        project["rear"].update(mechanical_trail=-.017, caster_deg=-7.)
        for direct in (False, True):
            if direct:
                project["hardpoints"] = {"enabled": True, "corners": hardpoints.from_scene(Engine(project).scene)}
                # A hardpoint project must use its geometry, not this unused draft.
                project["front"]["mechanical_trail"] = -.02
            engine = Engine(project)
            metrics = engine.solve_sv(0, 0)["metrics"]
            for axle in ("front", "rear"):
                with self.subTest(direct=direct, axle=axle):
                    self.assertAlmostEqual(metrics[f"trail_{axle}_mm"],
                                           engine.derived_parameters[axle]["mechanical_trail"]*1000, places=8)

    def test_sv_dynamic_trail_uses_current_road_and_caster(self):
        zero = self.engine.solve_sv(0, 0)["metrics"]
        states = [self.engine.solve_sv(15, -10), self.engine.solve_sv(-10, 20),
                  self.engine.sv_pose(4, -.6), self.engine.solve_sv(10, -5, wheel_center=True)]
        self.assertNotAlmostEqual(states[0]["metrics"]["trail_front_mm"], zero["trail_front_mm"], places=5)
        for state in states:
            metrics = state["metrics"]
            for axle in ("front", "rear"):
                with self.subTest(travel=(metrics["travel_front_mm"], metrics["travel_rear_mm"]), axle=axle):
                    # Upright motion preserves the perpendicular distance from
                    # the material contact reference to the kingpin axis.
                    expected = (zero[f"trail_{axle}_mm"]*math.cos(math.radians(zero[f"caster_{axle}_deg"]))
                                /math.cos(math.radians(metrics[f"caster_{axle}_deg"])))
                    self.assertAlmostEqual(metrics[f"trail_{axle}_mm"], expected, places=8)
                    points, frame = state["points"], state["frame"]
                    kg = to_contact_frame(points[axle+"_kg"], frame)
                    kt = to_contact_frame(points[axle+"_kt"], frame)
                    contact = to_contact_frame(points[axle+"_contact"], frame)
                    hit_x = kg[0]-kg[1]*(kt[0]-kg[0])/(kt[1]-kg[1])
                    sign = 1 if axle == "front" else -1
                    self.assertAlmostEqual(metrics[f"trail_{axle}_mm"], sign*(contact[0]-hit_x)*1000, places=8)

    def test_sv_trail_is_undefined_when_kingpin_is_parallel_to_current_ground(self):
        motion = self.engine.sv["front"]
        side = copy.deepcopy(motion.side(0))
        side["kg"] += [0., .2]
        side["kt"] = side["kg"]+[1., 0.]
        with patch.object(motion, "side", return_value=side):
            state = self.engine.solve_sv(0, 0)
        self.assertIsNone(state["metrics"]["trail_front_mm"])
        self.assertIsNotNone(state["metrics"]["trail_rear_mm"])

    def test_we_uses_static_axis_and_distinguishes_material_from_contact(self):
        for axle, corner, caster in (("front", "FR", 5), ("rear", "RR", 0)):
            initial = self.engine.solve_we(0, axle)
            np.testing.assert_allclose(initial["points"]["kingpin_ground"], self.engine.scene.geometry["kingpins"][corner]["ground"])
            for steering in (-30, 0, 30):
                s = self.engine.solve_we(steering, axle)
                self.assertAlmostEqual(s["metrics"]["caster_deg"], caster, places=10)
                self.assertAlmostEqual(s["metrics"]["material_radius_mm"], initial["metrics"]["material_radius_mm"], places=9)
                np.testing.assert_allclose(s["points"]["center"], self.engine.solve_we(steering, axle, caster)["points"]["center"], atol=1e-12)
                c, p = np.array(s["points"]["center"]), np.array(s["points"]["contact"])
                self.assertAlmostEqual(np.linalg.norm(c-p), self.engine.car.load_Radius[0 if axle == "front" else 1], places=10)
            self.assertNotAlmostEqual(initial["metrics"]["contact_radius_mm"], self.engine.solve_we(30, axle)["metrics"]["contact_radius_mm"], places=3)

    def test_mapping_masks_failures_and_has_all_requested_metrics(self):
        p = default_project()
        p["analysis"].update(samples=5, bump_mm=200, droop_mm=200)
        e = Engine(p)
        mapping = calculate_mapping(e, "front")
        self.assertEqual(mapping["total"], 25)
        self.assertTrue(any(not r["valid"] for r in mapping["rows"]))
        self.assertTrue(any(r["valid"] for r in mapping["rows"]))
        for page, mode in (("front", "pose"), ("rear", "travel"), ("sv", "travel"), ("sv", "pose"), ("we", "travel")):
            mapping = calculate_mapping(e, page, mode)
            for plot, _ in PLOTS[page]:
                if plot != "motion":
                    self.assertIsNotNone(make_plot(page, plot, mapping=mapping))
        self.assertIsNone(calculate_mapping(e, "front", cancelled=lambda: True))
        names = [r["name"] for r in static_point_rows(e)]
        self.assertEqual(len(names), len(set(names)))
        self.assertIn("FR_UBJ", names)


class ContactCoordinateTests(unittest.TestCase):
    def test_contact_coordinates_are_invariant_under_rigid_world_motion(self):
        first, second, point = np.array([-.6, .02]), np.array([.65, .08]), np.array([.12, .4])
        frame = contact_frame(first, second)
        basis = np.array([frame["x_axis"], frame["y_axis"]])
        np.testing.assert_allclose(basis@basis.T, np.eye(2), atol=1e-12)
        distance = np.linalg.norm(second-first)
        np.testing.assert_allclose(to_contact_frame(first, frame), [-distance/2, 0], atol=1e-12)
        np.testing.assert_allclose(to_contact_frame(second, frame), [distance/2, 0], atol=1e-12)
        matrix, shift = rotate2(.38), np.array([3., -2.])
        moved = contact_frame(matrix@first+shift, matrix@second+shift)
        np.testing.assert_allclose(to_contact_frame(matrix@point+shift, moved), to_contact_frame(point, frame), atol=1e-12)

    def test_rc_and_pc_use_contact_midpoint_and_perpendicular_height(self):
        e = Engine(default_project())
        for page in ("front", "rear", "sv"):
            for a, b in ((0, 0), (15, 15), (-10, 20), (20, -10)):
                with self.subTest(page=page, travel=(a, b)):
                    state = e.solve_sv(a, b) if page == "sv" else e.fv[page].solve(a, b)
                    p = state["points"]
                    first, second = (p["front_contact"], p["rear_contact"]) if page == "sv" else (p["L_gnd"], p["R_gnd"])
                    first, second = np.array(first), np.array(second)
                    u = (second-first)/np.linalg.norm(second-first)
                    n = np.array([-u[1], u[0]])
                    prefix, key = ("pc", "PC") if page == "sv" else ("rc", "RC")
                    delta = np.array(p[key])-(first+second)/2
                    m = state["metrics"]
                    self.assertAlmostEqual(m[prefix+"_x_prime_mm"], np.dot(delta, u)*1000, places=8)
                    self.assertAlmostEqual(m[prefix+"_y_prime_mm"], np.dot(delta, n)*1000, places=8)
                    np.testing.assert_allclose([m[prefix+"_world_x_mm"], m[prefix+"_world_z_mm"]], np.array(p[key])*1000)
                    zero = e.solve_sv(0, 0) if page == "sv" else e.fv[page].solve(0, 0)
                    np.testing.assert_allclose(p[key+"0"], zero["points"][key], atol=1e-12)
                    np.testing.assert_allclose(p["CG"], zero["points"]["CG"])
                    row = next(r for r in point_rows(state) if r["name"] == key)
                    self.assertAlmostEqual(row["x_prime_m"]*1000, m[prefix+"_x_prime_mm"])
                    self.assertAlmostEqual(row["y_prime_m"]*1000, m[prefix+"_y_prime_mm"])

    def test_fv_position_surfaces_and_distribution_use_rotating_frame(self):
        project = default_project()
        project["analysis"]["samples"] = 5
        e = Engine(project)
        for page in ("front", "rear"):
            self.assertIn("rc_x_prime_mm", dict(PLOTS[page]))
            self.assertIn("rc_y_prime_mm", dict(PLOTS[page]))
            mapping = calculate_mapping(e, page)
            for metric in ("rc_x_prime_mm", "rc_y_prime_mm"):
                spec = make_plot(page, metric, mapping=mapping)
                for grid_row, mesh_row in zip(mapping["grid"], spec["mesh"]):
                    self.assertEqual([r.get(metric) for r in grid_row], [p[2] for p in mesh_row])
            distribution = make_plot(page, "rc_map", mapping=mapping)
            self.assertEqual((distribution["xlabel"], distribution["ylabel"]), ("x′ (mm)", "y′ (mm)"))
            self.assertEqual(distribution["series"][0]["x"], [r["rc_x_prime_mm"] for r in mapping["grid"][0]])
            self.assertEqual(distribution["series"][0]["y"], [r["rc_y_prime_mm"] for r in mapping["grid"][0]])

    def test_undefined_rc_is_not_rendered_as_a_stale_point(self):
        e = Engine(default_project())
        with patch("suspension_geometry_lab.engine.intersection", return_value=None):
            state = e.fv["front"].solve(0, 0)
        self.assertIsNone(state["metrics"]["rc_x_prime_mm"])
        self.assertIsNone(state["metrics"]["rc_y_prime_mm"])
        self.assertNotIn("RC", state["points"])
        markers = make_plot("front", "motion", state)["markers"]
        self.assertNotIn("RC", {m["name"] for m in markers})
        self.assertIn("RC0", {m["name"] for m in markers})


class ChartScaleTests(unittest.TestCase):
    def test_equal_geometry_scale_survives_resize_zoom_and_pose_updates(self):
        from suspension_geometry_lab.charts import LabChart
        chart = object.__new__(LabChart)
        chart.canvas = Mock()
        chart.canvas.winfo_width.return_value = 1000
        chart.canvas.winfo_height.return_value = 470
        chart.spec = chart.view = None
        chart.primitives = chart.polygons = []
        chart.azimuth, chart.elevation = .65, .6
        engine = Engine(default_project())
        for page in ("front", "rear", "sv", "we", "static"):
            with self.subTest(page=page):
                state = (engine.fv[page].solve(0, 0) if page in ("front", "rear") else
                         engine.solve_sv(0, 0) if page == "sv" else engine.solve_we(0))
                chart.set(make_plot(page, "iso" if page == "static" else "motion", state, engine=engine))
                def scale():
                    l, t, r, b = chart.bounds
                    xmin, xmax, ymin, ymax = chart.view
                    xscale, yscale = (r-l)/(xmax-xmin), (b-t)/(ymax-ymin)
                    self.assertAlmostEqual(xscale, yscale, places=9)
                    return xscale
                scale()
                l, t, r, b = chart.bounds
                chart.zoom(SimpleNamespace(x=(l+r)/2, y=(t+b)/2, delta=120))
                expected_scale = scale()
                centre = ((chart.view[0]+chart.view[1])/2, (chart.view[2]+chart.view[3])/2)
                for width, height in ((650, 350), (1250, 610), (1000, 470)):
                    chart.canvas.winfo_width.return_value = width
                    chart.canvas.winfo_height.return_value = height
                    chart.draw()
                    self.assertAlmostEqual(scale(), expected_scale, places=9)
                    np.testing.assert_allclose([(chart.view[0]+chart.view[1])/2, (chart.view[2]+chart.view[3])/2], centre, atol=1e-12)
                chart.set(chart.spec, preserve_view=True)
                self.assertAlmostEqual(scale(), expected_scale, places=9)

    def test_round_point_markers_and_axes_are_present_in_svg_and_png(self):
        from io import BytesIO
        import xml.etree.ElementTree as ET
        from PIL import Image
        from suspension_geometry_lab.charts import LabChart
        chart = object.__new__(LabChart)
        chart.canvas = Mock()
        chart.canvas.winfo_width.return_value = 1200
        chart.canvas.winfo_height.return_value = 600
        chart.spec = chart.view = None
        chart.primitives = []
        chart.azimuth, chart.elevation = .65, .6
        state = Engine(default_project()).fv["front"].solve(-10, 20)
        spec = make_plot("front", "motion", state)
        # Reference axes and point labels survive hiding force lines.
        spec["world_lines"] = [p for p in spec["world_lines"] if not p["construction"]]
        chart.set(spec)
        self.assertEqual({c[-1] for c in chart.circles}, {"RC", "RC0", "CG", "IC L", "IC R"})
        labels = {p[4] for p in chart.primitives if p[0] == "text"}
        self.assertTrue({"x′", "y′"} <= labels)
        root = ET.fromstring(chart.svg())
        circles = root.findall("{http://www.w3.org/2000/svg}circle")
        self.assertEqual(len(circles), 5)
        original = next(c for c in circles if c.attrib["data-label"] == "RC0")
        self.assertEqual(original.attrib["fill"], "none")
        stream = BytesIO()
        stream.name = "points.png"
        chart.png(stream)
        cg = next(c for c in chart.circles if c[-1] == "CG")
        with Image.open(stream) as im:
            self.assertEqual(im.getpixel((round(cg[0]*2), round(cg[1]*2))), (251, 191, 36))
        view = chart.view.copy()
        chart.set(make_plot("front", "motion", Engine(default_project()).fv["front"].solve(15, -5)), preserve_view=True)
        self.assertEqual(chart.view, view)


@unittest.skipUnless(os.environ.get("SUSPENSION_GEOMETRY_LAB_GUI_TEST") == "1", "Opt-in native window tests")
class GuiTests(unittest.TestCase):
    def setUp(self):
        from suspension_geometry_lab.gui import App
        p = default_project()
        p["analysis"]["samples"] = 5
        self.app = App(p)
        self.app.update()

    def tearDown(self):
        self.app.dirty = False
        self.app.close()
        self.app.worker_thread.join(timeout=5)
        self.assertFalse(self.app.worker_thread.is_alive())
        self.app = None
        # Tcl variables must be finalized on the main thread between apps.
        gc.collect()

    def wait(self, panel):
        end = time.monotonic()+20
        while not getattr(panel, "ready_for_export", False) and time.monotonic() < end:
            self.app.update()
            time.sleep(.01)
        self.assertTrue(panel.ready_for_export, self.app.status.get())
        self.assertIsNone(panel.error)
        self.app.update()

    def test_tv_steering_previews_saves_and_exports_each_axle(self):
        panel = self.app.steering_panel
        self.app.notebook.select(panel)
        self.app.geometry("1150x760")
        self.app.update()
        self.assertEqual(panel.pane.winfo_class(), "TPanedwindow")
        left, right = map(str, panel.pane.panes())
        self.assertEqual(panel.scroll.winfo_parent(), left)
        self.assertTrue(panel.scroll.canvas.winfo_ismapped())
        self.assertEqual(panel.apply_button.winfo_parent(), left)
        self.assertTrue(panel.apply_button.winfo_ismapped())
        self.assertEqual(panel.chart.winfo_parent(), right)
        for key in ("tie_rod_x", "tie_rod_y", "rack_x", "rack_z", "rack_length",
                    "rack_limit_left_mm", "rack_limit_right_mm"):
            self.assertTrue(str(panel.entries[key]).startswith(f"{panel.scroll.content}."), key)
        self.assertEqual(panel.travel_frame.winfo_parent(), right)
        for widget in (panel.scale, panel.entries["rack_travel_mm"], panel.zero_button):
            self.assertTrue(str(widget).startswith(f"{panel.travel_frame}."))
            self.assertTrue(widget.winfo_ismapped())
        self.assertLessEqual(panel.travel_frame.winfo_rooty() + panel.travel_frame.winfo_height(),
                             panel.chart.winfo_rooty())
        self.assertIn("TV", self.app.notebook.tab(panel, "text"))
        self.assertTrue(self.app.export_buttons["state"].instate(["!disabled"]))
        self.assertTrue(self.app.export_buttons["mapping"].instate(["!disabled"]))
        self.assertTrue(panel.chart.spec["equal"])
        self.assertIn("tie_rod_x", panel.entries)
        self.assertIn("tie_rod_y", panel.entries)
        self.assertNotIn("tie_rod_z", panel.entries)
        self.assertAlmostEqual(float(panel.tie_rod_z.get()), panel.state["metrics"]["tie_rod_z"], places=6)
        self.assertIsNone(panel.state["metrics"]["ackermann_pct"])

        panel.chart.reset()
        view = panel.chart.view.copy()
        panel.scale.set(20.)
        end = time.monotonic() + 5
        while not panel.ready_for_export and time.monotonic() < end:
            self.app.update()
            time.sleep(.01)
        self.assertTrue(panel.ready_for_export, panel.notice.get())
        self.assertEqual(panel.state["metrics"]["rack_travel_mm"], 20.)
        self.assertLess(panel.state["metrics"]["right_steer_deg"], panel.state["metrics"]["left_steer_deg"])
        self.assertEqual(panel.metrics.item("ackermann_pct", "values")[3],
                         f"{panel.state['metrics']['ackermann_pct']:.2f}")
        self.assertEqual(panel.chart.view, view)
        self.assertFalse(self.app.dirty)
        old_engine = self.app.engine
        panel.inputs["rack_z"].set(".181")
        panel.apply()
        self.assertIs(self.app.engine, old_engine)
        self.assertTrue(self.app.dirty)

        static_panel = self.app.panels["static"]
        self.app.notebook.select(static_panel)
        self.app.update()
        self.wait(static_panel)
        links = [path["points"] for path in static_panel.chart.spec["world_lines"] if path["name"] == "Tie rod"]
        self.assertEqual(len(links), 4)
        front = self.app.project["steering"]["front"]
        self.assertNotIn("tie_rod_z", front)
        _, front_pickups = self.app.engine.centered_steering_pickups("front", front)
        self.assertIn([point.tolist() for point in front_pickups["L"]], links)
        self.app.notebook.select(panel)
        self.app.update()

        panel.axle.set("rear")
        panel.change_axle()
        self.assertEqual(panel.state["metrics"]["rack_travel_mm"], 0.)
        self.assertFalse(panel.rear_enabled.get())
        self.assertTrue(str(panel.rear_check).startswith(f"{panel.scroll.content}."))
        self.assertEqual(panel.travel_frame.winfo_manager(), "")
        panel.rear_check.invoke()
        self.assertTrue(panel.rear_enabled.get())
        self.assertEqual(panel.travel_frame.winfo_manager(), "pack")
        panel.inputs["rack_limit_right_mm"].set("12")
        panel.inputs["rack_travel_mm"].set("-10")
        panel.apply()
        self.assertEqual(panel.state["metrics"]["axle"], "rear")
        self.assertEqual(self.app.project["steering"]["front"]["rack_travel_mm"], 20.)
        self.assertTrue(self.app.project["steering"]["rear"]["enabled"])
        panel.inputs["rack_travel_mm"].set("1000")
        panel.apply()
        self.assertIsNone(panel.state)
        self.assertFalse(panel.ready_for_export)
        self.assertEqual(self.app.project["steering"]["rear"]["rack_travel_mm"], -10.)

        with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
            project_path = Path(tmp) / "steering.json"
            with patch("suspension_geometry_lab.gui.filedialog.asksaveasfilename", return_value=str(project_path)):
                self.assertTrue(self.app.save())
            self.app.import_project(project_path)
            self.assertEqual(panel.state["metrics"]["rack_travel_mm"], -10.)
            self.assertTrue(panel.ready_for_export)
            self.assertTrue(panel.rear_enabled.get())
            self.assertEqual(panel.scale.cget("to"), -12.)
            csv_path = Path(tmp) / "steering.csv"
            with patch("suspension_geometry_lab.gui.filedialog.asksaveasfilename", return_value=str(csv_path)):
                self.app.export("state")
            with csv_path.open(encoding="utf-8-sig", newline="") as handle:
                row = next(csv.DictReader(handle))
            self.assertEqual(row["axle"], "rear")
            self.assertEqual(float(row["rack_travel_mm"]), -10.)
            self.assertAlmostEqual(float(row["ackermann_pct"]), panel.state["metrics"]["ackermann_pct"])
            self.assertAlmostEqual(float(row["left_camber_gain_deg_per_mm"]),
                                   panel.state["metrics"]["left_camber_gain_deg_per_mm"])
            _, rear_pickups = self.app.engine.centered_steering_pickups("rear", self.app.project["steering"]["rear"])
            self.assertAlmostEqual(float(row["tie_rod_z"]), rear_pickups["L"][0][2])

            panel.rear_check.invoke()
            self.assertEqual(panel.travel_frame.winfo_manager(), "")
            self.assertTrue(panel.scale.instate(["disabled"]))
            self.assertEqual(panel.state["metrics"]["rack_travel_mm"], 0.)
            self.assertEqual(panel.state["metrics"]["left_steer_deg"], 0.)
            self.assertEqual(panel.state["metrics"]["right_steer_deg"], 0.)
            panel.apply()
            self.assertTrue(self.app.save())
            self.app.import_project(project_path)
            self.assertFalse(panel.rear_enabled.get())
            self.assertEqual(panel.travel_frame.winfo_manager(), "")
            self.assertEqual(self.app.project["steering"]["rear"]["rack_travel_mm"], 0.)
            panel.rear_check.invoke()
            self.assertEqual(panel.scale.cget("to"), -12.)
            self.assertEqual(panel.state["metrics"]["rack_travel_mm"], 0.)

    def test_tv_rack_slider_uses_separate_limits_and_rejects_typed_overtravel(self):
        panel = self.app.steering_panel
        self.app.notebook.select(panel)
        self.app.update()
        self.assertEqual(panel.rear_check.winfo_manager(), "")
        self.assertEqual((panel.scale.cget("to"), panel.scale.cget("from")), (-30., 30.))
        panel.inputs["rack_limit_left_mm"].set("22.125")
        limit_entry = panel.entries["rack_limit_right_mm"]
        panel.inputs["rack_limit_right_mm"].set("-1")
        self.assertEqual(limit_entry.cget("style"), "Invalid.TEntry")
        self.assertIn("請修正輸入", limit_entry.range_label.cget("text"))
        panel.inputs["rack_limit_right_mm"].set("10.775")
        self.assertEqual(limit_entry.cget("style"), "TEntry")
        panel.request()
        self.assertEqual((panel.scale.cget("to"), panel.scale.cget("from")), (-10.775, 22.125))
        for overtravel, expected in ((100., 22.125), (-100., -10.775)):
            panel.scale.set(overtravel)
            panel.request()
            self.assertAlmostEqual(panel.state["metrics"]["rack_travel_mm"], expected)

        panel.inputs["rack_travel_mm"].set("-10.78")
        panel.apply()
        self.assertIn("超出允許範圍", panel.notice.get())
        self.assertFalse(panel.ready_for_export)
        self.assertFalse(self.app.dirty)
        panel.inputs["rack_limit_left_mm"].set("0")
        panel.inputs["rack_limit_right_mm"].set("0")
        panel.zero_travel()
        self.assertTrue(panel.scale.instate(["disabled"]))
        self.assertEqual(panel.state["metrics"]["rack_travel_mm"], 0.)

    def test_tv_switches_sweep_charts_overlays_imported_lap_and_exports_camber_gains(self):
        panel = self.app.steering_panel
        self.app.notebook.select(panel)
        self.app.update()

        def wait_for(condition):
            deadline = time.monotonic() + 10
            while not condition() and time.monotonic() < deadline:
                self.app.update()
                time.sleep(.01)
            self.assertTrue(condition(), panel.plot_notice.get())

        with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
            sweep = Path(tmp) / "tv-sweep.csv"
            with patch("suspension_geometry_lab.gui.filedialog.asksaveasfilename", return_value=str(sweep)):
                self.app.export("mapping")
            wait_for(sweep.is_file)
            with sweep.open(encoding="utf-8-sig", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual((float(rows[0]["rack_travel_mm"]), float(rows[-1]["rack_travel_mm"])), (-30., 30.))
            turned = next(row for row in rows if math.isclose(float(row["rack_travel_mm"]), 20.))
            self.assertNotEqual(float(turned["left_camber_gain_deg_per_mm"]),
                                float(turned["right_camber_gain_deg_per_mm"]))

        self.app.import_lap(Path(__file__).resolve().parents[1] / "projects" / "lap_result.csv")
        panel.plot_selector.current(1)
        panel.plot_selector.event_generate("<<ComboboxSelected>>")
        wait_for(lambda: bool((panel.chart.spec or {}).get("lap_points")))
        self.assertEqual(panel.plot_key, "steer")
        self.assertEqual([line["name"] for line in panel.chart.spec["series"]], ["左輪 FL", "右輪 FR"])
        self.assertEqual(panel.chart.spec["lap_total"], len(self.app.lap_data.travels))
        self.assertTrue(panel.lap_check.instate(["!disabled"]))
        mapping = panel.mapping
        panel.scale.set(20.)
        panel.request()
        self.assertIs(panel.mapping, mapping)
        self.assertEqual(panel.chart.spec["markers"][0]["position"][0], 20.)
        values = panel.metrics.item("camber_gain_deg_per_mm", "values")
        self.assertNotEqual(values[1], values[2])

        panel.plot_selector.current(2)
        panel.plot_selector.event_generate("<<ComboboxSelected>>")
        wait_for(lambda: bool(panel.chart.spec and panel.chart.spec["ylabel"] == "阿克曼率 [%]"))
        self.assertTrue(panel.chart.spec["lap_points"])
        for index, field in ((3, "camber_deg"), (4, "camber_gain_deg_per_mm"),
                             (5, "steering_camber_gain_deg_per_deg")):
            panel.plot_selector.current(index)
            panel.plot_selector.event_generate("<<ComboboxSelected>>")
            self.assertTrue(panel.chart.spec["lap_points"])
            self.assertEqual(len(panel.chart.spec["series"]), 2)
            self.assertEqual(panel.chart.spec["markers"][0]["position"],
                             [20., panel.state["metrics"][f"left_{field}"]])
        panel.lap_check.invoke()
        self.assertNotIn("lap_points", panel.chart.spec)
        panel.plot_selector.current(0)
        panel.plot_selector.event_generate("<<ComboboxSelected>>")
        self.assertTrue(panel.chart.spec["equal"])

    def test_pacejka_tab_updates_with_geometry_and_exports(self):
        panel = self.app.tire_panel
        panel.corner.set("FR")
        self.app.notebook.select(panel)
        self.app.update()
        self.assertTrue(panel.ready_for_export, panel.notice.get())
        self.assertIn("Pacejka", self.app.notebook.tab(panel, "text"))
        self.assertTrue(self.app.step_button.instate(["disabled"]))
        self.assertTrue(self.app.export_buttons["points"].instate(["disabled"]))
        panel.inputs["FR"]["slip_angle_deg"].set("3.125")
        panel.inputs["FR"]["steer_deg"].set("10")
        self.assertFalse(panel.ready_for_export)
        panel.request()
        self.assertTrue(panel.ready_for_export, panel.notice.get())
        self.assertEqual(panel.state["metrics"]["slip_angle_deg"], 3.125)
        self.assertIn("輪胎 Mz", [series["name"] for series in panel.chart.spec["series"]])
        peaks = {marker["name"]: marker for marker in panel.chart.spec["markers"]}
        for label, extreme in (("正向峰值", max), ("負向峰值", min)):
            row = extreme(panel.mapping["rows"], key=lambda row: row["mz_nm"])
            self.assertEqual(peaks[label]["position"], [row["slip_angle_deg"], row["mz_nm"]])
            self.assertIn(f'data-label="{label}"', panel.chart.svg())

        before = panel.state["metrics"]
        project = copy.deepcopy(self.app.project)
        project["front"]["mechanical_trail"] += .01
        self.app.apply_project(project)
        after = panel.state["metrics"]
        self.assertAlmostEqual(before["mz_nm"], after["mz_nm"])
        self.assertNotAlmostEqual(before["torque_total_nm"], after["torque_total_nm"])
        with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
            path = Path(tmp) / "tire.csv"
            with patch("suspension_geometry_lab.gui.filedialog.asksaveasfilename", return_value=str(path)):
                self.app.export("mapping")
            with path.open(encoding="utf-8-sig", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), len(panel.mapping["rows"]))
            current = next(row for row in rows if float(row["slip_angle_deg"]) == 3.125)
            self.assertAlmostEqual(float(current["torque_total_nm"]), after["torque_total_nm"])

    def test_pacejka_front_axle_displays_and_exports_independent_inputs(self):
        from suspension_geometry_lab.tire_page import FRONT_ALIGNING_PLOT, FRONT_KINGPIN_PLOT

        panel = self.app.tire_panel
        self.app.notebook.select(panel)
        self.app.update()
        self.assertTrue(panel.ready_for_export, panel.notice.get())
        self.assertEqual(panel.visible_corners, ("FL", "FR"))
        for corner, steer, alpha, load in (("FL", "9", "2.5", "640"), ("FR", "7", "3.75", "975")):
            panel.inputs[corner]["steer_deg"].set(steer)
            panel.inputs[corner]["slip_angle_deg"].set(alpha)
            panel.inputs[corner]["fz_n"].set(load)
        panel.inputs["FL"]["wheel_camber_deg"].set("-2")
        panel.inputs["FR"]["wheel_camber_deg"].set("+3")
        panel.request()
        self.assertTrue(panel.ready_for_export, panel.notice.get())
        self.assertIn("右輪自動鏡射", panel.source_side.get())
        self.assertEqual(panel.metrics.heading("front", "text"), "前軸合計")
        current = panel.state["metrics"]
        self.assertEqual(current["fl_fz_n"], 640.)
        self.assertEqual(current["fr_fz_n"], 975.)
        self.assertEqual(current["fl_model_slip_angle_deg"], 2.5)
        self.assertEqual(current["fr_model_slip_angle_deg"], -3.75)
        self.assertEqual(current["fl_wheel_camber_deg"], -2.)
        self.assertEqual(current["fr_wheel_camber_deg"], 3.)
        self.assertEqual(panel.chart.spec["markers"][0]["position"], [0., current["front_aligning_total_nm"]])
        self.assertEqual(len(panel.chart.spec["series"]), 3)
        peaks = {marker["name"]: marker for marker in panel.chart.spec["markers"]}
        for label, extreme in (("正向峰值", max), ("負向峰值", min)):
            row = extreme(panel.mapping["rows"], key=lambda row: row["front_aligning_total_nm"])
            self.assertEqual(peaks[label]["position"], [row["slip_offset_deg"], row["front_aligning_total_nm"]])
        with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
            path = Path(tmp) / "front.csv"
            with patch("suspension_geometry_lab.gui.filedialog.asksaveasfilename", return_value=str(path)):
                self.app.export("state")
            with path.open(encoding="utf-8-sig", newline="") as handle:
                row = next(csv.DictReader(handle))
            self.assertAlmostEqual(float(row["front_mz_nm"]), current["front_mz_nm"])
            self.assertAlmostEqual(float(row["front_aligning_total_nm"]), current["front_aligning_total_nm"])
            self.assertEqual(float(row["fl_wheel_camber_deg"]), -2.)
            self.assertEqual(float(row["fr_wheel_camber_deg"]), 3.)
            self.assertAlmostEqual(float(row["front_torque_total_nm"]), current["front_torque_total_nm"])
        panel.plot.set(FRONT_KINGPIN_PLOT)
        self.app.geometry("1150x760")
        self.app.update()
        panel.draw_curve()
        peaks = {marker["name"]: marker for marker in panel.chart.spec["markers"]}
        self.assertEqual(len(peaks), 7)
        self.assertEqual(peaks["目前工況"]["position"], [0., current["front_torque_total_nm"]])
        for sign, label, extreme in ((1, "正向峰值", max), (-1, "負向峰值", min)):
            values = panel.metrics.item(f"kingpin-peak-{sign}", "values")
            for index, (prefix, name) in enumerate((("fl", "左前輪 FL"), ("fr", "右前輪 FR"),
                                                    ("front", "前軸合計")), start=1):
                key = f"{prefix}_torque_total_nm"
                row = extreme(panel.mapping["rows"], key=lambda row: row[key])
                angle, torque = row["slip_offset_deg"], row[key]
                marker_name = f"{name} {label}"
                self.assertEqual(peaks[marker_name]["position"], [angle, torque])
                self.assertTrue(peaks[marker_name]["label_lines"][0].startswith(f"{torque:+.4g} N·m"))
                self.assertIn(f">{peaks[marker_name]['label_lines'][0]}</text>", panel.chart.svg())
                self.assertIn(f'data-label="{marker_name}"', panel.chart.svg())
                self.assertTrue(values[index].startswith(f"{torque:+.4g} @ {angle:+.4g}°"))
        boxes = [coords for kind, coords, *_ in panel.chart.primitives if kind == "rect"]
        self.assertEqual(len(boxes), 7)
        for index, (left, top, right, bottom) in enumerate(boxes):
            for other_left, other_top, other_right, other_bottom in boxes[index+1:]:
                self.assertTrue(right <= other_left or left >= other_right or
                                bottom <= other_top or top >= other_bottom, (boxes[index], boxes))

        panel.plot.set(FRONT_ALIGNING_PLOT)
        panel.draw_curve()
        self.assertFalse(panel.metrics.exists("kingpin-peak-1"))
        for corner in ("FL", "FR"):
            panel.inputs[corner]["fz_n"].set("0")
        panel.request()
        self.assertTrue(panel.ready_for_export)
        self.assertEqual([marker["name"] for marker in panel.chart.spec["markers"]], ["目前工況"])

    def test_pacejka_preview_updates_each_wheel_before_calculation(self):
        panel = self.app.tire_panel
        self.app.notebook.select(panel)
        self.app.update()
        panel.inputs["FL"]["wheel_camber_deg"].set("-5")
        panel.inputs["FR"]["wheel_camber_deg"].set("-5")
        panel.inputs["FL"]["slip_angle_deg"].set("3")
        self.app.update()

        self.assertFalse(panel.ready_for_export)
        self.assertEqual(panel.preview.wheels["FL"].slip, 3.)
        self.assertEqual(panel.preview.wheels["FR"].slip, 0.)
        canvas = panel.preview.canvas
        texts = [canvas.itemcget(item, "text") for item in canvas.find_all() if canvas.type(item) == "text"]
        self.assertEqual(texts.count("Camber -5.00°"), 2)
        self.assertEqual(texts.count("輪頂向內傾"), 2)
        self.assertIn("行進方向", texts)

        panel.inputs["FL"]["steer_deg"].set("-")
        panel.inputs["FR"]["wheel_camber_deg"].set("+5")
        self.app.update()
        self.assertIsInstance(panel.preview.wheels["FL"], str)
        self.assertEqual(panel.preview.wheels["FR"].camber, 5.)
        panel.corner.set("FR")
        panel.request()
        self.assertTrue(panel.ready_for_export, panel.notice.get())
        self.assertEqual(tuple(panel.preview.wheels), ("FR",))
        self.assertAlmostEqual(panel.preview.wheels["FR"].camber, panel.state["metrics"]["wheel_camber_deg"])

    def test_lap_csv_import_toggle_and_cross_page_overlay(self):
        from suspension_geometry_lab.lap_data import TRAVEL_COLUMNS
        panel = self.app.panels["front"]
        panel.plot.set(dict(PLOTS["front"])["camber_left_deg"])
        panel.request()
        self.wait(panel)
        self.assertTrue(panel.lap_check.instate(["disabled"]))
        view = panel.chart.view.copy()
        with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
            path = Path(tmp) / "lap_result.csv"
            header = ",".join(TRAVEL_COLUMNS)
            path.write_text(header + "\n-16,10,-8,4\n-100,0,-100,0\n", encoding="utf-8-sig")
            with patch("suspension_geometry_lab.gui.filedialog.askopenfilename", return_value=str(path)):
                self.app.open_lap()
            self.wait(panel)
            self.assertEqual(self.app.lap_data.travels[0], (16, -10, 8, -4))
            self.assertFalse(self.app.lap_data.poses)
            self.assertEqual(panel.chart.spec["lap_total"], 2)
            self.assertEqual(len(panel.chart.spec["lap_points"]), 1)
            self.assertIn('data-label="LapSim"', panel.chart.svg())
            panel.lap_check.invoke()
            self.assertNotIn("lap_points", panel.chart.spec)
            self.assertNotIn('data-label="LapSim"', panel.chart.svg())
            panel.lap_check.invoke()
            self.assertEqual(panel.chart.view, view)
            self.assertIn('data-label="LapSim"', panel.chart.svg())
            path.write_text(header + "\nNaN,0,0,0\n", encoding="utf-8")
            with patch("suspension_geometry_lab.gui.messagebox.showerror") as error:
                self.app.import_lap(path)
            error.assert_called_once()
            self.assertEqual(self.app.lap_data.travels[0], (16, -10, 8, -4))

        for key, plot in (("rear", "rc_map"), ("sv", "anti_brake_front_pct")):
            panel = self.app.panels[key]
            self.app.notebook.select(panel)
            self.app.update()
            panel.plot.set(dict(PLOTS[key])[plot])
            panel.request()
            self.wait(panel)
            self.assertEqual(len(panel.chart.spec["lap_points"]), 1)
            self.assertIn('data-label="LapSim"', panel.chart.svg())
        side = self.app.engine.solve_sv(3, 2, wheel_center=True)["metrics"]
        np.testing.assert_allclose(panel.chart.spec["lap_points"][0],
                                   [side["ride_height_front_mm"], side["ride_height_rear_mm"],
                                    side["anti_brake_front_pct"]])
        panel.lap_check.invoke()
        for key in ("front", "rear", "sv"):
            self.assertNotIn("lap_points", self.app.panels[key].chart.spec)

    def test_lap_camber_chart_and_csv_follow_axle_and_source_file(self):
        from suspension_geometry_lab.analysis import LAP_CAMBER_PLOT
        from suspension_geometry_lab.lap_data import POSE_COLUMNS, TRAVEL_COLUMNS
        with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
            path = Path(tmp) / "lap_result.csv"
            header = ",".join((*TRAVEL_COLUMNS, *POSE_COLUMNS, "ay_g"))
            path.write_text(header + "\n12,-16,8,-10,0,3,0,1\n-16,12,-10,8,2,-3,1,-1\n0,0,0,0,0,0,0,0\n",
                            encoding="utf-8-sig")
            self.app.import_lap(path)
            for key, prefix in (("front", "F"), ("rear", "R")):
                panel = self.app.panels[key]
                self.app.notebook.select(panel)
                self.app.update()
                self.assertIn(LAP_CAMBER_PLOT[1], panel.plot_select["values"])
                panel.plot.set(LAP_CAMBER_PLOT[1])
                panel.request()
                self.wait(panel)
                self.assertEqual(len(panel.chart.spec["series"]), 4)
                checks = [widget for widget in panel.camber_filter.winfo_children()
                          if widget.winfo_class() == "TCheckbutton"]
                self.assertEqual(len(checks), 4)
                self.assertEqual(panel.chart.spec["hidden_series"], [])
                checks[0].invoke()
                self.assertEqual(panel.chart.spec["hidden_series"], ["內側 · 初始Camber＋車身姿態"])
                self.assertEqual(panel.mapping["straight"], 1)
                panel.lap_check.invoke()
                self.assertEqual(len(panel.chart.spec["series"]), 4)
                output = Path(tmp) / (key + ".csv")
                self.app.perform_export("mapping", output, panel)
                with output.open(encoding="utf-8-sig", newline="") as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual([row["inner_wheel"] for row in rows], [prefix+"L", prefix+"R"])
                self.assertEqual([row["ay_g"] for row in rows], ["1.0", "-1.0"])

            path.write_text(header + "\n0,0,0,0,2,4,0,1\n", encoding="utf-8-sig")
            self.app.import_lap(path)
            self.wait(panel)
            self.assertEqual(panel.mapping["valid"], 1)
            self.assertAlmostEqual(panel.mapping["rows"][0]["inner_camber_before_deg"], -4)
            self.assertLess(panel.chart.view[0], -4)
            self.assertGreater(panel.chart.view[1], 4)

            path.write_text(",".join(TRAVEL_COLUMNS) + "\n0,0,0,0\n", encoding="utf-8-sig")
            self.app.import_lap(path)
            self.wait(panel)
            self.assertEqual(panel.plot_key, "motion")
            self.assertNotIn(LAP_CAMBER_PLOT[1], panel.plot_select["values"])

    def test_we_lap_histogram_has_six_paired_series_and_restores_the_normal_axle(self):
        from suspension_geometry_lab.analysis import LAP_CAMBER_PLOT
        from suspension_geometry_lab.lap_data import FRONT_STEER_COLUMNS, POSE_COLUMNS, TRAVEL_COLUMNS
        panel = self.app.panels["we"]
        self.app.notebook.select(panel)
        self.app.update()
        panel.axle.set("rear")
        self.assertFalse(panel.camber_filter.winfo_manager())
        with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
            path = Path(tmp) / "lap_result.csv"
            columns = (*TRAVEL_COLUMNS, *POSE_COLUMNS, "ay_g")
            pose_rows = ("12,-16,8,-10,0,3,0,1", "-16,12,-10,8,2,-3,1,-1")
            path.write_text(",".join((*columns, *FRONT_STEER_COLUMNS))
                            + f"\n{pose_rows[0]},16,8\n{pose_rows[1]},-8,-16\n", encoding="utf-8-sig")
            self.app.import_lap(path)
            self.assertIn(LAP_CAMBER_PLOT[1], panel.plot_select["values"])
            panel.plot.set(LAP_CAMBER_PLOT[1])
            panel.request()
            self.wait(panel)
            self.assertFalse(panel.axle_controls.winfo_manager())
            self.assertTrue(panel.sliders[0].instate(["disabled"]))
            spec = panel.chart.spec
            self.assertEqual([sum(series["y"]) for series in spec["series"]], [2]*6)
            self.assertTrue(all(b-a == .25 for a, b in zip(spec["bin_edges"], spec["bin_edges"][1:])))
            self.assertTrue(panel.camber_filter.winfo_viewable())
            checks = [widget for widget in panel.camber_filter.winfo_children()
                      if widget.winfo_class() == "TCheckbutton"]
            labels = [f"{side} · {stage}" for side in ("內側", "外側")
                      for stage in ("初始Camber＋車身姿態", "姿態＋懸吊", "姿態＋懸吊＋轉向")]
            self.assertEqual([widget.cget("text") for widget in checks], labels)
            for label in labels:
                self.assertIn(label, panel.chart.svg())
            view, token = panel.chart.view.copy(), panel.token
            checks[-1].invoke()
            self.assertEqual(panel.chart.spec["hidden_series"], ["外側 · 姿態＋懸吊＋轉向"])
            self.assertEqual(panel.chart.spec["bin_edges"], spec["bin_edges"])
            self.assertEqual(panel.chart.view, view)
            self.assertEqual(panel.token, token)
            output = Path(tmp) / "we.csv"
            self.app.perform_export("mapping", output, panel)
            with output.open(encoding="utf-8-sig", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([row["inner_wheel"] for row in rows], ["FL", "FR"])
            self.assertEqual([row["inner_steer_deg"] for row in rows], ["16.0", "-16.0"])
            self.assertIn("outer_camber_steered_deg", rows[0])

            panel.plot.set(PLOTS["we"][0][1])
            panel.request()
            self.assertFalse(panel.camber_filter.winfo_manager())
            panel.plot.set(LAP_CAMBER_PLOT[1])
            panel.request()
            self.wait(panel)
            self.assertEqual(panel.chart.spec["hidden_series"], ["外側 · 姿態＋懸吊＋轉向"])

            path.write_text(",".join(columns) + "\n" + "\n".join(pose_rows) + "\n", encoding="utf-8")
            self.app.import_lap(path)
            self.wait(panel)
            self.assertEqual(panel.plot_key, "motion")
            self.assertNotIn(LAP_CAMBER_PLOT[1], panel.plot_select["values"])
            self.assertEqual(panel.axle.get(), "rear")
            self.assertTrue(panel.axle_controls.winfo_manager())
            self.assertFalse(panel.sliders[0].instate(["disabled"]))
            self.assertFalse(panel.camber_filter.winfo_manager())

    def test_import_prompts_for_missing_cells_and_commits_only_valid_repair(self):
        previous = self.app.project
        revision = self.app.revision
        source = copy.deepcopy(previous)
        source["vehicle"].update(mass=None, cg_height="")
        source["arm_mount_x"]["front"].pop("upper_front_x")
        with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
            path = Path(tmp) / "incomplete.json"
            path.write_text(json.dumps(source), encoding="utf-8")
            before = path.read_bytes()
            self.app.path = Path(tmp) / "previous.json"

            with patch.object(self.app, "wait_window", side_effect=lambda dialog: dialog.destroy()), \
                 patch("suspension_geometry_lab.gui.filedialog.askopenfilename", return_value=str(path)):
                self.app.open_project()
            self.assertIs(self.app.project, previous)
            self.assertEqual(self.app.path.name, "previous.json")
            self.assertEqual(self.app.revision, revision)

            def repair(dialog):
                self.assertEqual([field.path for field in dialog.fields],
                                 [("vehicle", "cg_height"), ("vehicle", "mass"),
                                  ("arm_mount_x", "front", "upper_front_x")])
                width, height = dialog.minsize()
                dialog.geometry(f"{width}x{height}")
                dialog.update()
                dialog.canvas.yview_moveto(1)
                dialog.update()
                button = dialog.apply_button
                self.assertTrue(button.winfo_viewable())
                self.assertLessEqual(button.winfo_rooty() + button.winfo_height(),
                                     dialog.canvas.winfo_rooty())
                dialog.inputs[1].insert(0, "10")
                button.invoke()
                self.assertTrue(dialog.error.cget("text"))
                self.assertIs(self.app.project, previous)
                self.assertEqual(self.app.revision, revision)
                dialog.inputs[1].delete(0, "end")
                dialog.inputs[1].insert(0, "325")
                dialog.inputs[2].insert(0, "400")
                button.invoke()
                self.assertIn("極限", dialog.error.cget("text"))
                self.assertIs(self.app.project, previous)
                dialog.inputs[2].delete(0, "end")
                button.invoke()

            with patch.object(self.app, "wait_window", side_effect=repair), \
                 patch("suspension_geometry_lab.gui.filedialog.askopenfilename", return_value=str(path)):
                self.app.open_project()
            self.assertEqual(self.app.project["vehicle"]["mass"], 325)
            self.assertEqual(self.app.project["vehicle"]["cg_height"], default_project()["vehicle"]["cg_height"])
            self.assertTrue(self.app.dirty)
            self.assertIsNone(self.app.path)
            self.assertEqual(path.read_bytes(), before)

    def test_every_plot_exports_real_png_svg_csv_report(self):
        with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
            for key, panel in self.app.panels.items():
                self.app.notebook.select(panel)
                self.app.update()
                for plot, label in PLOTS[key]:
                    panel.plot.set(label)
                    panel.request()
                    self.wait(panel)
                    self.assertIsNotNone(panel.chart.spec)
                    self.assertTrue(panel.note.winfo_ismapped())
                    self.assertLessEqual(panel.note.winfo_y()+panel.note.winfo_height(), panel.note.master.winfo_height())
                    self.assertTrue(panel.metrics.winfo_ismapped())
                    svg = panel.chart.svg()
                    self.assertIn("<svg", svg)
                    if "mesh" in panel.chart.spec:
                        self.assertIn("<polygon", svg)
                    if key != "static" and plot != "motion":
                        self.assertTrue(panel.chart.spec["markers"])
                        self.assertTrue(panel.chart.circles)
                    png = Path(tmp)/f"{key}_{plot}.png"
                    panel.chart.png(png)
                    self.assertGreater(png.stat().st_size, 1000)
                for kind, suffix in (("state", "csv"), ("points", "csv"), ("report", "html"), ("svg", "svg")):
                    path = Path(tmp)/f"{key}_{kind}.{suffix}"
                    self.app.perform_export(kind, path, panel)
                    self.assertTrue(path.exists())
                if key != "static":
                    path = Path(tmp)/f"{key}_mapping.csv"
                    self.app.perform_export("mapping", path, panel)
                    with path.open(encoding="utf-8-sig", newline="") as handle:
                        self.assertEqual(len(list(csv.DictReader(handle))), panel.mapping["total"])

    def test_step_button_exports_selected_page_even_from_analysis_chart(self):
        with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
            for key, panel in self.app.panels.items():
                with self.subTest(page=key):
                    self.app.notebook.select(panel)
                    self.app.update()
                    if key == "we":
                        self.assertTrue(self.app.step_button.instate(["disabled"]))
                        continue
                    if key != "static":
                        panel.plot.set(PLOTS[key][1][1])
                        panel.request()
                    self.wait(panel)
                    path = Path(tmp) / f"{key}.step"
                    with patch("suspension_geometry_lab.gui.filedialog.asksaveasfilename", return_value=str(path)) as save, \
                            patch.object(self.app, "wait_window", side_effect=lambda dialog: dialog.accept()):
                        self.app.step_button.invoke()
                    self.assertEqual(save.call_args.kwargs["defaultextension"], ".step")
                    self.assertIn("GEOMETRIC_CURVE_SET", path.read_text(encoding="ascii"))

    def test_parameter_commit_invalidates_pages_and_rejects_invalid_geometry(self):
        panel = self.app.active
        self.wait(panel)
        self.assertAlmostEqual(panel.state["metrics"]["rc_y_prime_mm"], 35)
        p = copy.deepcopy(self.app.project)
        p["front"]["rc_height"] = .048
        self.app.apply_project(p)
        self.wait(panel)
        self.assertAlmostEqual(panel.state["metrics"]["rc_y_prime_mm"], 48)
        self.assertEqual(self.app.panels["static"].entries[("front", "rc_height")].get(), "0.048")
        rev = self.app.revision
        invalid = copy.deepcopy(p)
        invalid["front"]["svsa"] = 0
        with self.assertRaises(ValueError):
            self.app.apply_project(invalid)
        self.assertEqual(self.app.revision, rev)
        self.assertEqual(self.app.project, p)
        from suspension_geometry_lab.gui import ParameterEditor
        editor = ParameterEditor(self.app)
        self.app.update()
        self.assertEqual(len(editor.entries), sum(map(len, __import__('suspension_geometry_lab.project', fromlist=['FIELDS']).FIELDS.values())))
        editor.destroy()

    def test_sv_sliders_move_cg_with_body_in_current_contact_view(self):
        panel = self.app.panels["sv"]
        self.app.notebook.select(panel)
        self.app.update()
        panel.request()
        self.wait(panel)
        cg = lambda: np.array(next(marker["position"] for marker in panel.chart.spec["markers"] if marker["name"] == "CG"))
        initial, view = cg().copy(), panel.chart.view.copy()
        for var in panel.controls:
            var.set(15)
        panel.slider_changed()
        self.wait(panel)
        self.assertAlmostEqual(cg()[1], initial[1]-.015, places=8)
        self.assertEqual(panel.chart.view, view)
        self.assertAlmostEqual(panel.state["metrics"]["cg_height_mm"], cg()[1]*1000, places=8)
        self.assertIn("Current contact frame", panel.chart.spec["xlabel"])
        panel.controls[0].set(-10)
        panel.controls[1].set(20)
        panel.slider_changed()
        self.wait(panel)
        self.assertNotAlmostEqual(panel.state["metrics"]["pitch_deg"], 0)
        row = next(row for row in point_rows(panel.state) if row["name"] == "CG")
        np.testing.assert_allclose([row["x_prime_m"], row["y_prime_m"]], cg(), atol=1e-12)
        ground = next(path for path in panel.chart.spec["world_lines"] if path["name"] == "ground")
        np.testing.assert_allclose(np.array(ground["points"])[:, 1], 0, atol=1e-12)

    def test_sv_table_and_exports_include_live_front_rear_trail_alongside_caster(self):
        panel = self.app.panels["sv"]
        self.app.notebook.select(panel)
        self.app.update()
        panel.request()
        self.wait(panel)

        def check_table():
            values = dict(panel.metrics.item(row, "values") for row in panel.metrics.get_children())
            for key in ("caster_front_deg", "trail_front_mm", "caster_rear_deg", "trail_rear_mm"):
                self.assertIn(RESULT_LABELS[key], values)
                self.assertAlmostEqual(float(values[RESULT_LABELS[key]]), panel.state["metrics"][key], delta=1e-5)
            return values

        before = check_table()
        panel.controls[0].set(15)
        panel.controls[1].set(-10)
        panel.slider_changed()
        self.wait(panel)
        self.assertNotEqual(check_table()[RESULT_LABELS["trail_front_mm"]], before[RESULT_LABELS["trail_front_mm"]])
        panel.mode.set("Heave / Pitch")
        panel.change_mode()
        self.wait(panel)
        check_table()
        with tempfile.TemporaryDirectory(dir=TEST_TMP) as temporary:
            state_path, report_path = Path(temporary)/"pose.csv", Path(temporary)/"report.html"
            self.app.perform_export("state", state_path, panel)
            self.app.perform_export("report", report_path, panel)
            with state_path.open(encoding="utf-8-sig", newline="") as handle:
                row, = csv.DictReader(handle)
            report = report_path.read_text(encoding="utf-8")
            for key in ("trail_front_mm", "trail_rear_mm"):
                self.assertAlmostEqual(float(row[key]), panel.state["metrics"][key])
                self.assertIn(RESULT_LABELS[key], report)

    def test_mapping_marker_follows_sliders_while_preserving_mesh_cache_and_camera(self):
        panel = self.app.panels["sv"]
        self.app.notebook.select(panel)
        self.app.update()
        panel.plot.set(dict(PLOTS["sv"])["caster_front_deg"])
        panel.request()
        self.wait(panel)
        initial_point = panel.chart.spec["markers"][0]["position"]
        np.testing.assert_allclose(initial_point, [50, 50, 5], atol=1e-10)
        self.assertIn("前／後行程: 0 / 0 mm", panel.chart.spec["markers"][0]["label_lines"])
        mapping, mesh = panel.mapping, copy.deepcopy(panel.chart.spec["mesh"])
        panel.chart.azimuth += .2
        panel.chart.draw()
        view, camera = panel.chart.view.copy(), (panel.chart.azimuth, panel.chart.elevation)
        panel.controls[0].set(15)
        panel.controls[1].set(-10)
        panel.slider_changed()
        self.wait(panel)
        self.assertIs(panel.mapping, mapping)
        self.assertEqual(panel.chart.spec["mesh"], mesh)
        self.assertEqual(panel.chart.view, view)
        self.assertEqual((panel.chart.azimuth, panel.chart.elevation), camera)
        self.assertEqual(len(panel.chart.circles), 1)
        np.testing.assert_allclose(panel.chart.spec["markers"][0]["position"],
                                   [35, 60, panel.state["metrics"]["caster_front_deg"]], atol=1e-8)
        self.assertNotEqual(initial_point, panel.chart.spec["markers"][0]["position"])
        panel.plot.set(dict(PLOTS["sv"])["pc_map"])
        panel.request()
        self.wait(panel)
        self.assertIs(panel.mapping, mapping)
        np.testing.assert_allclose(panel.chart.spec["markers"][0]["position"],
                                   [panel.state["metrics"]["pc_x_prime_mm"], panel.state["metrics"]["pc_y_prime_mm"]])
        self.assertEqual(len(panel.chart.circles), 1)

    def test_hardpoint_editor_mirrors_commits_and_renders_all_pages(self):
        from suspension_geometry_lab.gui import ParameterEditor
        from suspension_geometry_lab import hardpoints
        original = copy.deepcopy(self.app.project)
        editor = ParameterEditor(self.app)
        self.app.update()
        self.assertFalse(editor.hardpoint_mode.get())
        self.assertEqual(sum(len(editor.hardpoint_tree.get_children(c)) for c in hardpoints.CORNERS), 32)
        editor.hardpoint_check.invoke()
        self.assertEqual(str(editor.entry_widgets[("front", "kpi_deg")].cget("state")), "disabled")
        self.assertEqual(len(editor.hardpoint_tree.item("FR/RACK", "values")), 3)
        self.assertNotIn("RACK", editor.candidate()["hardpoints"]["corners"]["FR"])
        edited = (np.array(editor.point_data["FR"]["UBJ"]) + [-.005, .003, .002]).tolist()
        editor.set_hardpoint("FR", "UBJ", edited)
        np.testing.assert_array_equal(editor.point_data["FL"]["UBJ"], np.array(edited)*[1, -1, 1])
        rack = (np.array(editor.point_data["FR"]["RACK"]) + [.003, 0., 0.]).tolist()
        editor.set_hardpoint("FR", "RACK", rack)
        np.testing.assert_array_equal(editor.point_data["FL"]["RACK"], np.array(rack)*[1, -1, 1])
        self.assertEqual(self.app.project, original)
        self.assertEqual(len(editor.derived_tree.get_children()), len(DERIVED_AXLE_FIELDS))
        saved = editor.candidate()["hardpoints"]["corners"]
        self.assertIn("TOE", saved["FR"])
        self.assertNotIn("TOE", saved["RR"])
        editor.apply()
        self.assertFalse(editor.winfo_exists())
        self.assertTrue(hardpoints.enabled(self.app.project))
        self.assertEqual(self.app.engine.scene.geometry["hardpoints"], saved)
        self.assertEqual(self.app.project["front"], original["front"])
        self.assertEqual(str(self.app.panels["front"].entry_widgets[("front", "kpi_deg")].cget("state")), "disabled")
        steering = self.app.steering_panel
        self.assertTrue(steering.entries["rack_x"].instate(["readonly"]))
        self.assertFalse(steering.entries["rack_travel_mm"].instate(["readonly"]))
        self.assertAlmostEqual(float(steering.entries["rack_x"].get()), rack[0])
        self.assertEqual(steering.state["metrics"]["rack_x"], rack[0])
        # Disabled derived values must not overwrite the retained parameter design.
        self.app.panels["front"].apply()
        self.assertEqual(self.app.project["front"], original["front"])
        with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
            for key, panel in self.app.panels.items():
                self.app.notebook.select(panel)
                self.app.update()
                for plot, label in PLOTS[key]:
                    panel.plot.set(label)
                    panel.request()
                    self.wait(panel)
                    self.assertIn("<svg", panel.chart.svg())
                path = Path(tmp)/(key+".html")
                self.app.perform_export("report", path, panel)
                import html
                self.assertIn('"hardpoints"', html.unescape(path.read_text(encoding="utf-8")))
                panel.chart.png(Path(tmp)/(key+".png"))
        # Reopening keeps all edited XYZ; switching back restores parameter solving.
        editor = ParameterEditor(self.app)
        self.assertEqual(editor.candidate()["hardpoints"]["corners"], saved)
        editor.hardpoint_check.invoke()
        editor.apply()
        self.assertFalse(hardpoints.enabled(self.app.project))
        self.assertEqual(self.app.project["hardpoints"]["corners"], saved)
        self.assertAlmostEqual(self.app.engine.fv["front"].solve(0, 0)["metrics"]["rc_y_prime_mm"], 35)

    def test_invalid_hardpoint_draft_and_cancel_preserve_applied_project(self):
        from suspension_geometry_lab.gui import ParameterEditor
        original = copy.deepcopy(self.app.project)
        revision = self.app.revision
        editor = ParameterEditor(self.app)
        editor.entries[("front", "kpi_deg")].set("14")
        editor.hardpoint_check.invoke()
        self.assertAlmostEqual(float(editor.derived_tree.item("kpi_deg", "values")[0]), 14)
        editor.set_hardpoint("FL", "UCA_F", editor.point_data["FL"]["UCA_R"])
        self.assertIn("重合", editor.derivation_message.get())
        with patch("suspension_geometry_lab.gui.messagebox.showerror") as error:
            editor.apply()
        error.assert_called_once()
        self.assertEqual(self.app.revision, revision)
        self.assertEqual(self.app.project, original)
        self.assertTrue(editor.winfo_exists())
        editor.destroy()
        self.assertEqual(self.app.project, original)

    def test_parameter_preview_refreshes_vehicle_changes_without_applying(self):
        from suspension_geometry_lab.gui import ParameterEditor
        original = copy.deepcopy(self.app.project)
        editor = ParameterEditor(self.app)
        initial_anti = float(editor.derived_tree.item("anti_brake", "values")[0])

        def wait_for(condition):
            end = time.monotonic()+3
            while not condition() and time.monotonic() < end:
                self.app.update()
                time.sleep(.01)
            self.assertTrue(condition())

        bias = editor.entries[("vehicle", "brake_bias_front")]
        bias.set("0.5")
        expected = Engine(editor.candidate(True)).derived_parameters["front"]["anti_brake"]
        self.assertNotEqual(initial_anti, expected)
        wait_for(lambda: bool(editor.derived_tree.get_children()) and
                 abs(float(editor.derived_tree.item("anti_brake", "values")[0])-expected) < 1e-7)
        self.assertEqual(self.app.project, original)
        bias.set("")
        wait_for(lambda: "需要數值" in editor.derivation_message.get())
        bias.set("0.5")
        wait_for(lambda: "硬點幾何有效" in editor.derivation_message.get())
        self.assertEqual(self.app.project, original)
        with patch.object(self.app, "report_callback_exception") as callback_error:
            bias.set("0.4")
            editor.destroy()
            end = time.monotonic()+.3
            while time.monotonic() < end:
                self.app.update()
                time.sleep(.01)
            callback_error.assert_not_called()

    def test_mount_x_fields_preview_fallback_but_reject_invalid_commit(self):
        from suspension_geometry_lab.gui import ParameterEditor
        editor = ParameterEditor(self.app)
        self.assertEqual(len(editor.mount_x_entries), 8)
        original = copy.deepcopy(self.app.project)

        def wait_for(condition):
            end = time.monotonic()+3
            while not condition() and time.monotonic() < end:
                self.app.update()
                time.sleep(.01)
            self.assertTrue(condition(), editor.derivation_message.get())

        front = editor.mount_x_entries[("front", "upper_front_x")]
        rear = editor.mount_x_entries[("front", "upper_rear_x")]
        front.set("200")
        rear.set("-200")
        wait_for(lambda: abs(editor.point_data["FR"]["UCA_F"][0]-.2) < 1e-12)
        self.assertAlmostEqual(editor.point_data["FR"]["UCA_R"][0], -.2)
        self.assertIn("實際使用 X：200.000000 mm", editor.mount_x_labels[("front", "upper_front_x")].cget("text"))
        self.assertEqual(self.app.project, original)
        baseline = self.app.engine.scene.geometry["mount_position_checks"]["front"]["upper_front_x"]
        front.set(str((baseline["limit_max_x"]+.01)*1000))
        wait_for(lambda: "超過極限範圍 10 mm" in editor.mount_x_labels[("front", "upper_front_x")].cget("text"))
        self.assertAlmostEqual(editor.point_data["FR"]["UCA_F"][0], baseline["limit_max_x"])
        self.assertAlmostEqual(editor.point_data["FR"]["UCA_R"][0], baseline["limit_min_x"])
        self.assertEqual(float(rear.get()), -200)
        with patch("suspension_geometry_lab.gui.messagebox.showerror") as error:
            editor.apply()
        error.assert_called_once()
        self.assertTrue(editor.winfo_exists())
        self.assertEqual(self.app.project, original)
        front.set("")
        editor.apply()
        self.assertFalse(editor.winfo_exists())
        checks = self.app.engine.scene.geometry["mount_position_checks"]["front"]
        self.assertFalse(checks["upper_front_x"]["fallback"])
        self.assertIsNone(self.app.project["arm_mount_x"]["front"]["upper_front_x"])
        self.assertEqual(self.app.project["arm_mount_x"]["front"]["upper_rear_x"], -.2)
        editor = ParameterEditor(self.app)
        editor.hardpoint_check.invoke()
        self.assertTrue(all(str(entry.cget("state")) == "disabled" for entry in editor.mount_x_widgets.values()))
        editor.destroy()

    def test_split_travel_fields_and_auto_pose_ranges_replace_redundant_inputs(self):
        from suspension_geometry_lab.gui import ParameterEditor
        editor = ParameterEditor(self.app)
        self.assertIn(("analysis", "bump_mm"), editor.entries)
        self.assertIn(("analysis", "droop_mm"), editor.entries)
        self.assertNotIn(("analysis", "roll_deg"), editor.entries)
        self.assertNotIn(("analysis", "pitch_deg"), editor.entries)
        self.assertIn(("front", "svsa"), editor.entries)
        self.assertIn(("rear", "svsa"), editor.entries)
        self.assertIn(("vehicle", "drive_force_front"), editor.entries)
        self.assertIn(("front", "anti"), editor.entries)
        self.assertIn(("rear", "anti"), editor.entries)
        self.assertNotIn(("rear", "anti_dive"), editor.entries)
        self.assertTrue(editor.derived_tree.exists("svsa"))
        editor.entries[("analysis", "bump_mm")].set("40")
        editor.entries[("analysis", "droop_mm")].set("10")
        editor.apply()
        self.assertEqual(self.app.project["schema_version"], 7)
        for page in ("front", "rear", "sv"):
            panel = self.app.panels[page]
            self.app.notebook.select(panel)
            self.app.update()
            panel.request()
            self.wait(panel)
            for scale in panel.sliders:
                self.assertEqual(float(scale.cget("from")), -10)
                self.assertEqual(float(scale.cget("to")), 40)
            if page == "sv":
                self.assertIn(("front", "svsa"), panel.entries)
                self.assertIn(("vehicle", "drive_force_front"), panel.entries)
            panel.mode.set("Heave / Pitch" if page == "sv" else "Heave / Roll")
            panel.change_mode()
            self.wait(panel)
            self.assertEqual(float(panel.sliders[0].cget("from")), -10)
            self.assertEqual(float(panel.sliders[0].cget("to")), 40)
            if page == "sv":
                limit = self.app.engine.pitch_angle_limit()
                low, high = -limit, limit
            else:
                low, high = self.app.engine.fv[page].roll_bounds(0)
            self.assertAlmostEqual(float(panel.sliders[1].cget("from")), low)
            self.assertAlmostEqual(float(panel.sliders[1].cget("to")), high)

    def test_heave_roll_controls_shrink_to_each_wheel_stroke_and_surface_keeps_live_marker(self):
        project = copy.deepcopy(self.app.project)
        project["analysis"].update(bump_mm=40, droop_mm=10, samples=5)
        self.app.apply_project(project)
        for axle in ("front", "rear"):
            panel = self.app.panels[axle]
            self.app.notebook.select(panel)
            panel.mode.set("Heave / Roll")
            panel.plot.set(dict(PLOTS[axle])["camber_left_deg"])
            panel.change_mode()
            self.wait(panel)
            mapping = panel.mapping
            for heave in (15, 35, 40, -10, 0):
                panel.controls[0].set(heave)
                panel.controls[1].set(10.)  # Requests beyond stroke must clamp.
                panel.slider_changed()
                self.wait(panel)
                self.assertIsNone(panel.error)
                m = panel.state["metrics"]
                self.assertTrue(all(-10-1e-5 <= m[key] <= 40+1e-5 for key in ("travel_left_mm", "travel_right_mm")))
                self.assertLess(min(abs(m[key]-bound) for key in ("travel_left_mm", "travel_right_mm") for bound in (-10, 40)), 1e-5)
                self.assertIs(panel.mapping, mapping)
                point = panel.chart.spec["markers"][0]["position"]
                np.testing.assert_allclose(point, [heave, m["roll_deg"], m["camber_left_deg"]], atol=1e-6)
                self.assertEqual(panel.sliders[1].instate(["disabled"]), heave in (-10, 40))

    def test_shared_anti_and_svsa_keep_geometry_when_drive_share_is_edited(self):
        from suspension_geometry_lab.gui import ParameterEditor
        from suspension_geometry_lab.project import MOUNT_X_FIELDS
        project = copy.deepcopy(self.app.project)
        project["vehicle"]["drive_force_front"] = .25
        project["front"]["svsa"] = 2.5
        project["rear"]["svsa"] = 3.
        project["front"]["anti"] = .3
        project["rear"]["anti"] = .2
        project["arm_mount_x"] = {axle: {key: None for key in MOUNT_X_FIELDS} for axle in ("front", "rear")}
        project["arm_mount_x"]["front"]["upper_front_x"] = .2001234567890123
        self.app.apply_project(project)
        before = self.app.engine.solve_sv(0, 0)["metrics"]
        editor = ParameterEditor(self.app)
        for axle in ("front", "rear"):
            self.assertEqual({key for group, key in editor.entries if group == axle and key.startswith("anti")}, {"anti"})
            for key in ("anti", "svsa"):
                self.assertEqual(str(editor.entry_widgets[(axle, key)].cget("state")), "normal")
        value = float(editor.derived_tree.item("anti_drive", "values")[0])
        self.assertNotAlmostEqual(value, 0)
        # This tree previews the fixed hardpoint draft. Its Anti scales with
        # drive share, while the parameter-mode Anti design target stays fixed.
        preview_anti = value * .5 / .25
        editor.entries[("vehicle", "drive_force_front")].set("0.5")
        deadline = time.time()+3
        while time.time() < deadline:
            self.app.update()
            values = editor.derived_tree.item("anti_drive", "values")
            if values and abs(float(values[0])-preview_anti) < 1e-7:
                break
            time.sleep(.01)
        self.assertAlmostEqual(float(editor.derived_tree.item("anti_drive", "values")[0]), preview_anti, places=7)
        self.assertEqual(float(editor.entries[("front", "anti")].get()), .3)
        self.assertEqual(float(editor.entries[("vehicle", "brake_bias_front")].get()), .7)
        self.assertEqual(self.app.project["vehicle"]["drive_force_front"], .25)
        editor.apply()
        self.assertEqual(self.app.project["vehicle"]["drive_force_front"], .5)
        after = self.app.engine.solve_sv(0, 0)["metrics"]
        self.assertAlmostEqual(after["anti_drive_front_pct"], 15)
        self.assertAlmostEqual(after["anti_drive_rear_pct"], 10)
        self.assertEqual(after["svsa_front_mm"], before["svsa_front_mm"])
        self.assertEqual(self.app.project["arm_mount_x"], project["arm_mount_x"])
        self.assertAlmostEqual(after["anti_brake_rear_pct"], before["anti_brake_rear_pct"], places=10)
        panel = self.app.panels["sv"]
        self.app.notebook.select(panel)
        for key in ("anti_brake_front_pct", "anti_brake_rear_pct", "anti_drive_front_pct", "anti_drive_rear_pct"):
            panel.plot.set(dict(PLOTS["sv"])[key])
            panel.request()
            self.wait(panel)
            self.assertIsNone(panel.error)
            self.assertTrue(panel.chart.spec["markers"])

    def test_force_share_edits_preserve_anti_inputs_and_geometry(self):
        from suspension_geometry_lab.gui import ParameterEditor
        editor = ParameterEditor(self.app)
        previous = Engine(editor.candidate(False))
        for new in (.4, 0., 1., .3):
            editor.entries[("vehicle", "drive_force_front")].set(str(new))
            deadline = time.time()+3
            while time.time() < deadline:
                self.app.update()
                if editor._preview_timer is None:
                    break
                time.sleep(.01)
            current = Engine(editor.candidate(False))
            for axle in ("front", "rear"):
                for key in ("anti", "svsa"):
                    self.assertEqual(str(editor.entry_widgets[(axle, key)].cget("state")), "normal")
                np.testing.assert_array_equal(current.sv[axle].template.initial_ic, previous.sv[axle].template.initial_ic)
        editor.apply()
        panel = self.app.panels["sv"]
        self.app.notebook.select(panel)
        panel.entries[("front", "anti")].set("0.20")
        panel.entries[("front", "svsa")].set("3")
        panel.apply()
        front = self.app.engine.sv["front"].template
        np.testing.assert_allclose(front.initial_ic-front.contact_point,
                                   [3., .6*self.app.engine.car.h_cog/self.app.engine.car.l])
        self.wait(panel)
        metrics = dict(panel.metrics.item(item, "values") for item in panel.metrics.get_children())
        for key, value in (("anti_front_pct", 20), ("anti_brake_front_pct", 14), ("anti_drive_front_pct", 6)):
            self.assertAlmostEqual(float(metrics[ANTI_RESULT_LABELS[key]]), value)
        panel.controls[0].set(12.)
        panel.controls[1].set(-5.)
        panel.request()
        self.wait(panel)
        before = panel.state
        panel.entries[("vehicle", "drive_force_front")].set("1")
        for axle in ("front", "rear"):
            for key in ("anti", "svsa"):
                self.assertEqual(str(panel.entry_widgets[(axle, key)].cget("state")), "normal")
        panel.entries[("vehicle", "brake_bias_front")].set("0.5")
        panel.apply()
        self.assertEqual(self.app.project["vehicle"]["drive_force_front"], 1)
        self.assertEqual(self.app.engine.solve_sv(0, 0)["metrics"]["anti_drive_rear_pct"], 0)
        self.wait(panel)
        self.assertEqual([var.get() for var in panel.controls], [12., -5.])
        self.assertEqual(panel.state["points"], before["points"])
        metrics = dict(panel.metrics.item(item, "values") for item in panel.metrics.get_children())
        self.assertAlmostEqual(float(metrics[ANTI_RESULT_LABELS["anti_brake_front_pct"]]),
                               before["metrics"]["anti_brake_front_pct"]*.5/.7, places=5)
        panel.entries[("rear", "svsa")].set("3")
        panel.apply()
        self.assertEqual(self.app.project["rear"]["svsa"], 3)
        self.assertAlmostEqual(self.app.engine.derived_parameters["rear"]["svsa"], 3)

    def test_sliders_preserve_viewport_zoom_pan_and_3d_camera(self):
        for key in ("front", "rear", "sv", "we"):
            with self.subTest(page=key):
                panel = self.app.panels[key]
                self.app.notebook.select(panel)
                self.app.update()
                panel.request()
                self.wait(panel)
                chart = panel.chart
                initial_view = chart.view.copy()
                # Ordinary slider motion must not change the fitted axis limits.
                panel.controls[0].set(8.)
                panel.slider_changed()
                self.wait(panel)
                self.assertEqual(chart.view, initial_view)
                l, t, r, b = chart.bounds
                chart.zoom(SimpleNamespace(x=(l+r)/2, y=(t+b)/2, delta=120))
                chart.drag = (100, 100)
                chart.pan(SimpleNamespace(x=145, y=120))
                camera = (chart.view.copy(), chart.azimuth, chart.elevation)
                old_geometry = chart.spec["world_lines"]
                panel.controls[0].set(-10.)
                panel.slider_changed()
                self.wait(panel)
                self.assertNotEqual(chart.spec["world_lines"], old_geometry)
                self.assertEqual((chart.view, chart.azimuth, chart.elevation), camera)
                # A temporary unreachable pose must not discard the camera either.
                panel.receive(None, None, "unreachable pose")
                self.assertEqual(chart.view, camera[0])
                panel.request()
                self.wait(panel)
                self.assertEqual((chart.view, chart.azimuth, chart.elevation), camera)
                # Explicit reset still fits the current geometry and restores angles.
                chart.reset()
                self.assertNotEqual(chart.view, camera[0])
                self.assertEqual((chart.azimuth, chart.elevation), (.65, .6))

    def test_continuous_slider_movement_displays_intermediate_solutions(self):
        panel = self.app.panels["front"]
        self.wait(panel)
        original_solve = self.app.engine.fv["front"].solve
        def slow_solve(*args, **kwargs):
            # Even a solve slower than the input rate must produce live frames.
            time.sleep(.04)
            return original_solve(*args, **kwargs)
        seen = set()
        with patch.object(self.app.engine.fv["front"], "solve", side_effect=slow_solve):
            start = time.monotonic()
            while time.monotonic()-start < .8:
                value = -15+30*(time.monotonic()-start)/.8
                panel.sliders[0].set(value)
                self.app.update()
                seen.add(panel.state["metrics"]["travel_left_mm"])
                time.sleep(.008)
            # There has been no release or pause in the input stream yet.
            self.assertGreaterEqual(len(seen-{0.}), 2)
            panel.sliders[0].set(20.)
            self.wait(panel)
            self.assertAlmostEqual(panel.state["metrics"]["travel_left_mm"], 20.)
            self.assertFalse(panel._request_in_flight)

    def test_initial_page_fits_mapped_canvas_instead_of_hidden_canvas(self):
        for key in ("front", "rear", "sv", "we"):
            panel = self.app.panels[key]
            self.app.notebook.select(panel)
            self.app.update()
            panel.request()
            self.wait(panel)
            chart = panel.chart
            projection = chart.spec["projection"]
            points = [p for path in chart.spec["world_lines"] if not path["construction"] and path.get("fit", True)
                      for p in path["points"]]
            xs = [p[0] if len(p) == 2 else chart.project(p, projection)[0] for p in points]
            fraction = (max(xs)-min(xs))/(chart.view[1]-chart.view[0])
            # The tall 3D kingpin leaves more horizontal space than 2D views.
            self.assertGreater(fraction, .15 if key == "we" else .3, (key, fraction, chart.view))

    def test_static_reference_checkbox_hides_construction_preserving_camera(self):
        import html
        panel = self.app.panels["static"]
        self.app.notebook.select(panel)
        self.app.update()
        scene = self.app.engine.scene
        source_points = [path.points.copy() for path in scene.lines]
        for plot, label in PLOTS["static"]:
            panel.plot.set(label)
            panel.request()
            self.wait(panel)
            chart = panel.chart
            full_lines = copy.deepcopy(chart.spec["world_lines"])
            l, t, r, b = chart.bounds
            chart.zoom(SimpleNamespace(x=(l+r)/2, y=(t+b)/2, delta=120))
            camera = (chart.view.copy(), chart.azimuth, chart.elevation)
            panel.static_references_check.invoke()
            self.app.update()
            hidden = [path["points"] for path in chart.spec["world_lines"]]
            self.assertLess(len(hidden), len(full_lines))
            for wheel in scene.geometry["kingpins"].values():
                self.assertNotIn(np.array([wheel["ground"], wheel["axis_top"]]).tolist(), hidden)
                self.assertIn(np.array([wheel["LBJ"], wheel["UBJ"]]).tolist(), hidden)
            for plane in scene.geometry["arm_planes"].values():
                self.assertNotIn(np.vstack((plane["vertices"], plane["vertices"][0])).tolist(), hidden)
            self.assertEqual((chart.view, chart.azimuth, chart.elevation), camera)
            if plot == "iso":
                with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
                    report = Path(tmp)/"hidden-reference.html"
                    self.app.perform_export("report", report, panel)
                    self.assertIn('"static_references": false', html.unescape(report.read_text(encoding="utf-8")))
                    chart.png(Path(tmp)/"hidden-reference.png")
            panel.static_references_check.invoke()
            self.app.update()
            self.assertEqual(chart.spec["world_lines"], full_lines)
            self.assertEqual((chart.view, chart.azimuth, chart.elevation), camera)
        for path, original in zip(scene.lines, source_points):
            np.testing.assert_array_equal(path.points, original)


if __name__ == "__main__":
    unittest.main()
