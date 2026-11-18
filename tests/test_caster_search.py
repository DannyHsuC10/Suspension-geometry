"""Caster selection agrees with the plotted outer steered-camber normal fit."""
import copy
import csv
from dataclasses import replace
import gc
import math
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from suspension_geometry_lab import hardpoints
from suspension_geometry_lab.analysis import LAP_CAMBER_PLOT, make_plot
from suspension_geometry_lab.caster_search import (
    OUTER_STEERED_CAMBER, _caster_mapping, _fit_camber, caster_candidates,
    search_front_caster, validate_caster_lap,
)
from suspension_geometry_lab.engine import Engine
from suspension_geometry_lab.lap_data import (
    FRONT_STEER_COLUMNS, POSE_COLUMNS, TRAVEL_COLUMNS, LapData, LapPose, calculate_lap_camber,
)
from suspension_geometry_lab.project import default_project
from suspension_geometry_lab.roll_center_search import SearchCancelled


def sample_project():
    project = default_project()
    project["front"]["static_camber_deg"] = -1.5
    project["analysis"]["samples"] = 5
    return project


def sample_lap():
    return LapData(((-12., 16., 0., 0.), (16., -12., 0., 0.), (-4., 8., 0., 0.),
                    (8., -4., 0., 0.), (0., 0., 0., 0.), (100., 0., 0., 0.)),
                   (LapPose(0., 3., 1., .1), LapPose(0., -3., -1., -.1),
                    LapPose(1., 2., .5, .1), LapPose(1., -2., -.5, -.1),
                    LapPose(0., 0., 0., 0.), LapPose(0., 3., 0., .1)),
                   "curvature_1_m", ((16., 8.), (-8., -16.), (12., 6.), (-6., -12.), (0., 0.), (12., 9.)))


def write_lap(path, lap, *, steering=True):
    fields = [*TRAVEL_COLUMNS, *POSE_COLUMNS, lap.turn_column, *(FRONT_STEER_COLUMNS if steering else ())]
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        for travels, pose, angles in zip(lap.travels, lap.poses, lap.front_steer_deg):
            writer.writerow([*(-value for value in travels), pose.heave_mm, pose.roll_deg, pose.pitch_deg,
                             pose.turn_signal, *(angles if steering else ())])


class CasterSearchTests(unittest.TestCase):
    def test_full_grid_optimum_matches_uncached_plot_calculations(self):
        project, lap = sample_project(), sample_lap()
        original = copy.deepcopy(project)
        self.assertEqual(caster_candidates(), [index/10 for index in range(30, 61)])
        oracle = []
        for caster in caster_candidates():
            candidate = copy.deepcopy(project)
            candidate["front"]["caster_deg"] = caster
            mapping = calculate_lap_camber(Engine(candidate), "front", lap, include_steering=True)
            values = [row[OUTER_STEERED_CAMBER] for row in mapping["rows"]]
            oracle.append((abs(float(np.mean(values))), caster))
        result = search_front_caster(project, lap)
        self.assertEqual(result.best.caster_deg, min(oracle)[1])
        self.assertEqual(result.best.caster_deg, 3.1)
        self.assertEqual(result.tested, 31)
        self.assertLess(result.best.error_deg, result.before.error_deg)
        self.assertEqual((result.mapping["valid"], result.mapping["total"], result.mapping["straight"]), (4, 5, 1))
        curve = make_plot("we", LAP_CAMBER_PLOT[0], mapping=result.mapping)["normal_curves"][-1]
        self.assertAlmostEqual(curve["mean_deg"], result.best.mean_deg)
        self.assertAlmostEqual(curve["std_deg"], result.best.std_deg)
        self.assertEqual(curve["count"], result.best.samples)
        self.assertEqual(curve["x"][int(np.argmax(curve["y"]))], result.best.mean_deg)
        expected = copy.deepcopy(original)
        expected["front"]["caster_deg"] = result.best.caster_deg
        self.assertEqual(result.project, expected)
        self.assertEqual(project, original)

    def test_cached_fv_poses_match_uncached_steering_and_cache_unreachable_samples(self):
        project, lap, cache = sample_project(), sample_lap(), {}
        project["front"]["caster_deg"] = 3.
        first = Engine(project)
        calculate_lap_camber(first, "front", lap, include_steering=True, _wheel_center_cache=cache)
        self.assertIsNone(cache[("front", 100., 0.)])
        project["front"]["caster_deg"] = 6.
        engine = Engine(project)
        expected = calculate_lap_camber(engine, "front", lap, include_steering=True)
        with patch.object(engine.fv["front"], "wheel_center_sides", side_effect=AssertionError("Use cached FV poses")):
            cached = calculate_lap_camber(engine, "front", lap, include_steering=True, _wheel_center_cache=cache)
        self.assertEqual(cached, expected)

    def test_peak_is_raw_mean_not_absolute_mean_or_histogram_bin_mode(self):
        values = [-.11, .03, .11, .18, .37]
        fit = _fit_camber({"rows": [{OUTER_STEERED_CAMBER: value} for value in values]}, 4.)
        self.assertAlmostEqual(fit.mean_deg, sum(values)/len(values))
        self.assertNotAlmostEqual(fit.mean_deg, sum(map(abs, values))/len(values))
        self.assertAlmostEqual(fit.std_deg, float(np.std(values, ddof=0)))
        for invalid in ([], [1.], [1., 1.], [1., float("nan")]):
            with self.subTest(values=invalid), self.assertRaises(ValueError):
                _fit_camber({"rows": [{OUTER_STEERED_CAMBER: value} for value in invalid]}, 4.)

    def test_invalid_angles_do_not_stop_search_and_ties_choose_lower_caster(self):
        real_mapping = _caster_mapping

        def reject_short(project, lap, cache, cancelled):
            if project["front"]["caster_deg"] < 4.:
                raise ValueError("invalid geometry")
            return real_mapping(project, lap, cache, cancelled)

        with patch("suspension_geometry_lab.caster_search._caster_mapping", side_effect=reject_short):
            result = search_front_caster(sample_project(), sample_lap())
        self.assertEqual(result.tested, 31)
        self.assertEqual(result.rejected["invalid geometry"], 10)
        self.assertEqual(result.best.caster_deg, 4.)
        lap = replace(sample_lap(), front_steer_deg=((0., 0.),)*6)
        result = search_front_caster(sample_project(), lap)
        self.assertEqual(result.best.caster_deg, 3.)

    def test_changed_sample_subset_is_not_accepted(self):
        real_mapping = _caster_mapping

        def lose_samples(project, lap, cache, cancelled):
            mapping = real_mapping(project, lap, cache, cancelled)
            if project["front"]["caster_deg"] != 5.:
                mapping["rows"] = mapping["rows"][:-1]
            return mapping

        with patch("suspension_geometry_lab.caster_search._caster_mapping", side_effect=lose_samples):
            result = search_front_caster(sample_project(), sample_lap())
        self.assertEqual(result.best.caster_deg, 5.)
        self.assertEqual(sum(result.rejected.values()), 30)

    def test_missing_data_cancellation_hardpoints_and_invalid_mounts_do_not_mutate(self):
        project, lap = sample_project(), sample_lap()
        before = copy.deepcopy(project)
        for invalid in (replace(lap, front_steer_deg=()), replace(lap, front_steer_deg=((0., 0.),)),
                        replace(lap, poses=()), replace(lap, turn_column=None),
                        replace(lap, front_steer_deg=((float("nan"), 0.),)*6),
                        replace(lap, poses=tuple(replace(pose, turn_signal=0.) for pose in lap.poses))):
            with self.subTest(lap=invalid), self.assertRaises(ValueError):
                validate_caster_lap(invalid)
        with self.assertRaises(SearchCancelled):
            search_front_caster(project, lap, cancelled=lambda: True)
        event = threading.Event()
        with self.assertRaises(SearchCancelled):
            search_front_caster(project, lap, cancelled=event.is_set, progress=lambda *args: event.set())
        self.assertEqual(project, before)
        project["arm_mount_x"]["front"]["upper_front_x"] = .8
        failed = search_front_caster(project, lap)
        self.assertIsNone(failed.project)
        self.assertTrue(failed.rejected)
        project = copy.deepcopy(before)
        project["hardpoints"] = {"enabled": True, "corners": hardpoints.from_scene(Engine(project).scene)}
        with self.assertRaisesRegex(ValueError, "硬點"):
            search_front_caster(project, lap)


@unittest.skipUnless(os.environ.get("SUSPENSION_GEOMETRY_LAB_GUI_TEST") == "1", "Opt-in native window tests")
class CasterSearchGuiTests(unittest.TestCase):
    def setUp(self):
        from suspension_geometry_lab.gui import App
        self.app = App(sample_project())
        self.app.update()
        self.panel = self.app.panels["we"]
        self.app.notebook.select(self.panel)
        self.app.update()

    def wait_dialog(self, dialog):
        end = time.monotonic()+15
        while dialog.poll_timer is not None and time.monotonic() < end:
            self.app.update()
            time.sleep(.01)
        self.assertIsNone(dialog.poll_timer)

    def tearDown(self):
        self.app.dirty = False
        self.app.close()
        self.app.worker_thread.join(timeout=5)
        self.panel = None
        self.app = None
        gc.collect()

    def test_buttons_reuse_loaded_csv_and_current_parameter_drafts(self):
        lap = sample_lap()
        self.app.set_lap_data(lap, Path("loaded.csv"))
        for page in ("sv", "we"):
            panel = self.app.panels[page]
            panel.entries[("front", "caster_deg")].set("5.24")
            button = panel.entry_widgets[("front", "caster_deg")].action_button
            self.assertEqual(button.cget("text"), "尋找最佳Caster")
            with patch("suspension_geometry_lab.gui.filedialog.askopenfilename") as choose, \
                    patch("suspension_geometry_lab.gui.CasterSearchDialog") as search:
                button.invoke()
            choose.assert_not_called()
            owner, project, received_lap, path = search.call_args.args
            self.assertIs(owner, panel)
            self.assertIs(received_lap, lap)
            self.assertEqual(project["front"]["caster_deg"], 5.2)
            self.assertEqual(path, Path("loaded.csv"))
            panel.caster_search_dialog = None

    def test_file_import_real_search_applies_and_shows_winning_normal_curve(self):
        directory = Path(__file__).resolve().parents[1]/".test-tmp"
        directory.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=directory) as temporary:
            path = Path(temporary)/"lap.csv"
            write_lap(path, sample_lap())
            with patch("suspension_geometry_lab.gui.filedialog.askopenfilename", return_value=str(path)):
                self.panel.find_front_caster()
            dialog = self.panel.caster_search_dialog
            self.wait_dialog(dialog)
            end = time.monotonic()+15
            while not self.panel.ready_for_export and time.monotonic() < end:
                self.app.update()
                time.sleep(.01)
            self.assertIn("已套用", dialog.status.get())
            self.assertEqual(self.app.project["front"]["caster_deg"], 3.1)
            self.assertEqual(self.app.lap_path, path)
            self.assertEqual(self.panel.plot.get(), LAP_CAMBER_PLOT[1])
            self.assertTrue(self.panel.show_normal_curves.get())
            self.assertTrue(self.panel.camber_visible[OUTER_STEERED_CAMBER].get())
            curve = self.panel.chart.spec["normal_curves"][-1]
            self.assertAlmostEqual(curve["mean_deg"], .002447673104398543)
            self.assertEqual(curve["count"], 4)
            self.assertTrue(self.app.dirty)
            dialog.destroy()

    def test_missing_or_cancelled_csv_never_starts_search(self):
        with patch("suspension_geometry_lab.gui.filedialog.askopenfilename", return_value=""), \
                patch("suspension_geometry_lab.gui.CasterSearchDialog") as search:
            self.panel.find_front_caster()
        search.assert_not_called()
        directory = Path(__file__).resolve().parents[1]/".test-tmp"
        with tempfile.TemporaryDirectory(dir=directory) as temporary:
            path = Path(temporary)/"missing.csv"
            write_lap(path, sample_lap(), steering=False)
            with patch("suspension_geometry_lab.gui.filedialog.askopenfilename", return_value=str(path)), \
                    patch("suspension_geometry_lab.gui.CasterSearchDialog") as search, \
                    patch("suspension_geometry_lab.gui.messagebox.showerror") as error:
                self.panel.find_front_caster()
            search.assert_not_called()
            self.assertIn("steer_FL_deg", error.call_args.args[1])
        self.assertIsNone(self.app.lap_data)

    def test_no_result_stale_csv_revision_cancel_and_hardpoints_never_apply(self):
        from suspension_geometry_lab.caster_dialog import CasterSearchDialog
        before, lap = copy.deepcopy(self.app.project), sample_lap()
        self.app.set_lap_data(lap, Path("old.csv"))
        good = search_front_caster(before, lap)
        bad = replace(good, project=None, best=None, rejected={"無有效分布": 31})
        with patch.object(CasterSearchDialog, "run_search", return_value=None):
            dialog = CasterSearchDialog(self.panel, before, lap, "old.csv")
            dialog.finish(bad)
            self.assertIn("沒有可套用", dialog.status.get())
            self.app.revision += 1
            dialog.finish(good)
            self.assertIn("已變更", dialog.status.get())
            dialog.destroy()
            self.assertTrue(dialog.cancel_event.is_set())
            dialog = CasterSearchDialog(self.panel, before, lap, "old.csv")
            self.app.set_lap_data(replace(lap, turn_column="ay_g"), Path("new.csv"))
            dialog.finish(good)
            self.assertIn("已變更", dialog.status.get())
            dialog.destroy()
        self.assertEqual(self.app.project, before)
        before["hardpoints"] = {"enabled": True, "corners": hardpoints.from_scene(self.app.engine.scene)}
        self.app.apply_project(before)
        for page in ("sv", "we"):
            button = self.app.panels[page].entry_widgets[("front", "caster_deg")].action_button
            self.assertTrue(button.instate(["disabled"]))


if __name__ == "__main__":
    unittest.main()
