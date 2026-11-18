"""Lap-driven FVSA search must stop at the first failing increment."""
import copy
import gc
import math
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np

from suspension_geometry_lab import hardpoints
from suspension_geometry_lab.engine import Engine, Unreachable
from suspension_geometry_lab.fvsa_search import (
    CamberPeak, FVSASearchResult, _wheel_center_motion, calculate_lap_peak,
    fvsa_candidates, prepare_lap, search_fvsa, validate_camber_limit,
)
from suspension_geometry_lab.lap_data import LapData, LapPose, calculate_lap_camber, road_camber
from suspension_geometry_lab.project import default_project
from suspension_geometry_lab.roll_center_search import SearchCancelled


def sample_lap():
    return LapData(((-12., 16., -8., 10.), (16., -12., 10., -8.), (0., 0., 0., 0.)),
                   (LapPose(0., 3., 1., .1), LapPose(0., -3., 1., -.1), LapPose(2., 1., 0., 0.)),
                   "curvature_1_m")


def peak(value):
    return CamberPeak(value, -value, 1, "FL", 3)


class FVSASearchTests(unittest.TestCase):
    def test_batch_matches_scalar_solver_for_both_axles_and_fvsa_lengths(self):
        travels = np.array([-18.37, -11., -1., 0., 2.76, 13.6, 20.])
        for axle in ("front", "rear"):
            for length in (.625, .9, 2., 20.):
                with self.subTest(axle=axle, length=length):
                    project = default_project()
                    project[axle]["fvsa"] = length
                    motion = Engine(project).fv[axle]
                    expected = [motion.side(t, wheel_center=True)["angle"] for t in travels]
                    actual = _wheel_center_motion(motion, travels, lambda: False)
                    np.testing.assert_allclose(actual, expected, rtol=0., atol=1e-9)

    def test_peak_matches_existing_lap_camber_and_includes_straight_samples(self):
        lap = sample_lap()
        engine = Engine(default_project())
        for axle, prefix in (("front", "F"), ("rear", "R")):
            result = calculate_lap_peak(engine, axle, lap)
            reference = calculate_lap_camber(engine, axle, lap)["rows"]
            turn_peak = max(abs(row[f"{role}_camber_after_deg"]) for row in reference for role in ("inner", "outer"))
            self.assertAlmostEqual(result.peak_deg, max(turn_peak, 1.), places=8)
            self.assertEqual(result.total, 3)
            self.assertTrue(result.wheel.startswith(prefix))
        straight = LapData(((0., 0., 0., 0.),), (LapPose(0., 7., 0., 0.),), "ay_g")
        self.assertAlmostEqual(calculate_lap_peak(engine, "front", straight).peak_deg, 7., places=8)

    def test_negative_camber_is_limited_and_all_rows_are_checked(self):
        project = default_project()
        engine = Engine(project)
        lap = sample_lap()
        absolute = calculate_lap_peak(engine, "front", lap)
        self.assertEqual(absolute.peak_deg, abs(absolute.signed_deg))
        # Independent scalar road-frame calculation over every wheel/sample.
        values = []
        for travels, pose in zip(lap.travels, lap.poses):
            left, right, _ = engine.fv["front"].wheel_center_sides(*travels[:2])
            roll, pitch = math.radians(pose.roll_deg), math.radians(pose.pitch_deg)
            up = (math.sin(pitch), math.cos(pitch)*math.sin(roll), math.cos(pitch)*math.cos(roll))
            for side, state in ((1, left), (-1, right)):
                body = side*state["angle"]
                values.append(road_camber((0., side*math.cos(body), -math.sin(body)), up))
        self.assertAlmostEqual(absolute.peak_deg, max(map(abs, values)), places=8)
        self.assertAlmostEqual(calculate_lap_peak(engine, "front", lap, absolute=False).peak_deg, max(values), places=8)

    def test_exact_half_track_start_and_increment_do_not_round_half_away(self):
        project = default_project()
        project["vehicle"]["track_front"] = 1.255
        candidates = fvsa_candidates(project, "front")
        self.assertEqual(candidates[:3], [.6275, .6325, .6375])
        self.assertLessEqual(candidates[-1], 20.)
        self.assertGreater(candidates[-1]+.005, 20.)

    def test_first_exceed_stops_and_returns_previous_even_if_later_would_pass(self):
        project = default_project()
        before = copy.deepcopy(project)
        progress = []
        with patch("suspension_geometry_lab.fvsa_search._lap_peak",
                   side_effect=[peak(.5), peak(1.), peak(1.1), peak(.2)]) as evaluate:
            result = search_fvsa(project, "front", sample_lap(), 1., progress=lambda *value: progress.append(value))
        self.assertEqual(evaluate.call_count, 3)
        self.assertEqual(result.project["front"]["fvsa"], .630)
        self.assertEqual(result.stopped_fvsa_m, .635)
        self.assertEqual(result.stop_reason, "camber_limit")
        self.assertEqual(result.peak.peak_deg, 1.)  # Exact equality is allowed.
        self.assertEqual(result.tested, 3)
        self.assertEqual(len(progress), 3)
        self.assertEqual(project, before)

    def test_real_search_best_passes_and_next_increment_exceeds(self):
        project = default_project()
        lap = sample_lap()
        for axle in ("front", "rear"):
            with self.subTest(axle=axle):
                limit = 2. if axle == "front" else 2.5
                result = search_fvsa(project, axle, lap, limit)
                self.assertIsNotNone(result.project)
                self.assertEqual(result.stop_reason, "camber_limit")
                self.assertLessEqual(calculate_lap_peak(Engine(result.project), axle, lap).peak_deg, limit)
                after = copy.deepcopy(result.project)
                after[axle]["fvsa"] = result.stopped_fvsa_m
                self.assertGreater(calculate_lap_peak(Engine(after), axle, lap).peak_deg, limit)

    def test_start_failure_and_geometry_stop_never_skip_invalid_candidates(self):
        project = default_project()
        with patch("suspension_geometry_lab.fvsa_search._lap_peak", return_value=peak(3.)):
            result = search_fvsa(project, "front", sample_lap(), 2.)
        self.assertIsNone(result.project)
        self.assertEqual(result.tested, 1)
        with patch("suspension_geometry_lab.fvsa_search._lap_peak", side_effect=[peak(.8), Unreachable("無解"), peak(.9)]) as evaluate:
            result = search_fvsa(project, "front", sample_lap(), 2.)
        self.assertEqual(evaluate.call_count, 2)
        self.assertEqual(result.project["front"]["fvsa"], .625)
        self.assertEqual(result.stop_reason, "geometry")
        self.assertIn("無解", result.error)

    def test_upper_limit_is_reported_without_claiming_an_exceed(self):
        with patch("suspension_geometry_lab.fvsa_search.fvsa_candidates", return_value=[.625, .630]), \
                patch("suspension_geometry_lab.fvsa_search._lap_peak", return_value=peak(.8)):
            result = search_fvsa(default_project(), "front", sample_lap(), 2.)
        self.assertEqual(result.project["front"]["fvsa"], .630)
        self.assertEqual(result.stop_reason, "upper_limit")
        self.assertIsNone(result.stopped_peak)

    def test_missing_poses_unreachable_lap_and_manual_mounts_are_not_accepted(self):
        with self.assertRaisesRegex(ValueError, "車身姿態"):
            prepare_lap(LapData(((0., 0., 0., 0.),), (), None))
        lap = LapData(((0., 100., 0., 0.),), (LapPose(0., 0., 0., 0.),), "ay_g")
        result = search_fvsa(default_project(), "front", lap, 2.)
        self.assertIsNone(result.project)
        self.assertEqual(result.stop_reason, "geometry")
        project = default_project()
        project["arm_mount_x"]["front"]["upper_front_x"] = .8
        result = search_fvsa(project, "front", sample_lap(), 2.)
        self.assertIsNone(result.project)
        self.assertIn("A 臂", result.error)

    def test_cancellation_invalid_limits_and_hardpoints_preserve_source(self):
        project = default_project()
        before = copy.deepcopy(project)
        with self.assertRaises(SearchCancelled):
            search_fvsa(project, "front", sample_lap(), 2., cancelled=lambda: True)
        with self.assertRaises(SearchCancelled):
            calculate_lap_peak(Engine(project), "front", sample_lap(), cancelled=lambda: True)
        for value in (-1., 91., float("nan"), float("inf"), True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_camber_limit(value)
        self.assertEqual(project, before)
        project["hardpoints"] = {"enabled": True, "corners": hardpoints.from_scene(Engine(project).scene)}
        with self.assertRaisesRegex(ValueError, "硬點"):
            search_fvsa(project, "front", sample_lap(), 2.)


@unittest.skipUnless(os.environ.get("SUSPENSION_GEOMETRY_LAB_GUI_TEST") == "1", "Opt-in native window tests")
class FVSASearchGuiTests(unittest.TestCase):
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

    def test_buttons_require_file_then_limit_and_preserve_current_drafts(self):
        for axle in ("front", "rear"):
            panel = self.app.panels[axle]
            panel.entries[("vehicle", "track_"+axle)].set("1.255")
            button = panel.entry_widgets[(axle, "fvsa")].action_button
            self.assertEqual(button.cget("text"), "尋找最佳FVSA")
            with patch("suspension_geometry_lab.gui.filedialog.askopenfilename", return_value="lap.csv") as choose, \
                    patch("suspension_geometry_lab.gui.read_lap_data", return_value=sample_lap()), \
                    patch("suspension_geometry_lab.gui.simpledialog.askfloat", return_value=2.) as ask, \
                    patch("suspension_geometry_lab.gui.FVSASearchDialog") as dialog:
                button.invoke()
                choose.assert_called_once()
                ask.assert_called_once()
                owner, project, lap, path, limit = dialog.call_args.args
                self.assertIs(owner, panel)
                self.assertEqual(project["vehicle"]["track_"+axle], 1.255)
                self.assertEqual((path, limit), ("lap.csv", 2.))
                self.assertEqual(self.app.lap_data, lap)
            panel.fvsa_search_dialog = None

    def test_file_or_limit_cancel_and_invalid_csv_never_start_search(self):
        panel = self.app.panels["front"]
        before = copy.deepcopy(self.app.project)
        with patch("suspension_geometry_lab.gui.filedialog.askopenfilename", return_value=""), \
                patch("suspension_geometry_lab.gui.simpledialog.askfloat") as ask:
            panel.find_fvsa()
            ask.assert_not_called()
        with patch("suspension_geometry_lab.gui.filedialog.askopenfilename", return_value="lap.csv"), \
                patch("suspension_geometry_lab.gui.read_lap_data", return_value=sample_lap()), \
                patch("suspension_geometry_lab.gui.simpledialog.askfloat", return_value=None), \
                patch("suspension_geometry_lab.gui.FVSASearchDialog") as dialog:
            panel.find_fvsa()
            dialog.assert_not_called()
        with patch("suspension_geometry_lab.gui.filedialog.askopenfilename", return_value="bad.csv"), \
                patch("suspension_geometry_lab.gui.read_lap_data", side_effect=ValueError("缺少欄位")), \
                patch("suspension_geometry_lab.gui.messagebox.showerror") as error:
            panel.find_fvsa()
            error.assert_called_once()
        self.assertEqual(self.app.project, before)
        self.assertIsNone(self.app.lap_data)

    def test_real_search_imports_csv_applies_previous_value_and_updates_fields(self):
        panel = self.app.panels["front"]
        # Read an actual CSV, including a straight sample; do not use its
        # pre-existing camber fields as the geometry search criterion.
        directory = Path(__file__).resolve().parents[1]/".test-tmp"
        directory.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=directory) as temporary:
            path = Path(temporary)/"lap.csv"
            path.write_text("zs_minus_zu_FL_mm,zs_minus_zu_FR_mm,zs_minus_zu_RL_mm,zs_minus_zu_RR_mm,heave_mm,roll_deg,pitch_deg,ay_g\n"
                            "12,-16,8,-10,0,3,1,1\n-16,12,-10,8,0,-3,1,-1\n0,0,0,0,2,1,0,0\n", encoding="utf-8")
            with patch("suspension_geometry_lab.gui.filedialog.askopenfilename", return_value=str(path)), \
                    patch("suspension_geometry_lab.gui.simpledialog.askfloat", return_value=2.):
                panel.find_fvsa()
            dialog = panel.fvsa_search_dialog
            end = time.monotonic()+15
            while dialog.poll_timer is not None and time.monotonic() < end:
                self.app.update()
                time.sleep(.01)
            self.assertIsNone(dialog.poll_timer)
            self.assertIn("已套用", dialog.status.get())
            self.assertIn("下一級", dialog.status.get())
            self.assertTrue(self.app.dirty)
            self.assertEqual(float(panel.entries[("front", "fvsa")].get()), self.app.project["front"]["fvsa"])
            self.assertEqual(self.app.lap_path, path)
            self.assertLessEqual(calculate_lap_peak(self.app.engine, "front", self.app.lap_data).peak_deg, 2.)
            dialog.destroy()

    def test_no_result_stale_results_and_cancel_do_not_replace_project(self):
        from suspension_geometry_lab.fvsa_dialog import FVSASearchDialog
        panel = self.app.panels["front"]
        before = copy.deepcopy(self.app.project)
        dialog = FVSASearchDialog(panel, before, sample_lap(), "lap.csv", 0.)
        dialog.finish(FVSASearchResult(None, None, 1, 10, "camber_limit", .625, peak(3.)))
        self.assertIn("沒有可套用", dialog.status.get())
        best = copy.deepcopy(before)
        best["front"]["fvsa"] = .630
        self.app.revision += 1
        dialog.finish(FVSASearchResult(best, peak(1.), 2, 10, "upper_limit"))
        self.assertIn("專案已變更", dialog.status.get())
        dialog.destroy()
        self.assertTrue(dialog.cancel_event.is_set())
        self.assertEqual(self.app.project, before)
        project = copy.deepcopy(before)
        project["hardpoints"] = {"enabled": True, "corners": hardpoints.from_scene(self.app.engine.scene)}
        self.app.apply_project(project)
        for axle in ("front", "rear"):
            self.assertTrue(self.app.panels[axle].entry_widgets[(axle, "fvsa")].action_button.instate(["disabled"]))


if __name__ == "__main__":
    unittest.main()
