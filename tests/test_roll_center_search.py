"""RC search checks the full independent-wheel domain and applies atomically."""
import copy
import gc
import os
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from suspension_geometry_lab import hardpoints
from suspension_geometry_lab.engine import Engine, Unreachable
from suspension_geometry_lab.project import default_project
from suspension_geometry_lab.roll_center_search import (
    SearchCancelled, SearchSettings, check_travel, search_roll_center,
)


class SyntheticMotion:
    def __init__(self, slope=lambda travel: .05):
        self.slope = slope

    def side(self, travel):
        ground = np.array([.6, travel/1000])
        return {"gnd": ground, "IC": ground+[-1., self.slope(travel)],
                "kua": np.array([.5, .3]), "cpb_ru": np.array([.2, .2]),
                "lak": np.array([.5, .1]), "cpb_rl": np.array([.2, .1])}


class RollCenterSearchTests(unittest.TestCase):
    def test_vectorized_contacts_match_solver_for_all_asymmetric_pairs(self):
        engine = Engine(default_project())
        travels = np.array([-21.3, -5., 0., 7.8, 24.5])
        for axle in ("front", "rear"):
            motion = engine.fv[axle]
            metrics = [motion.solve(l, r, gains=False)["metrics"] for l in travels for r in travels]
            check = check_travel(motion, travels)
            self.assertTrue(check.valid, check.reason)
            self.assertAlmostEqual(check.min_height_mm, min(m["rc_y_prime_mm"] for m in metrics), places=9)
            self.assertAlmostEqual(check.max_abs_x_mm, max(abs(m["rc_x_prime_mm"]) for m in metrics), places=8)
            l, r = check.worst_travel_mm
            self.assertAlmostEqual(motion.solve(l, r, gains=False)["metrics"]["rc_y_prime_mm"],
                                   check.min_height_mm, places=9)

    def test_search_returns_first_feasible_step_without_mutating_source(self):
        project = default_project()
        before = copy.deepcopy(project)
        progress = []
        settings = SearchSettings(-5, 60, 25, 25)
        for axle in ("front", "rear"):
            result = search_roll_center(project, axle, settings, progress=lambda *args: progress.append(args))
            self.assertIsNotNone(result.project)
            self.assertTrue(result.check.valid)
            height = result.project[axle]["rc_height"]
            self.assertEqual(height, .034)
            previous = copy.deepcopy(result.project)
            previous[axle]["rc_height"] = height-.001
            self.assertFalse(check_travel(Engine(previous).fv[axle], settings.travels()).valid)
            self.assertEqual(result.travel_samples, 51)
            self.assertEqual(result.tested, 40)
        self.assertEqual(project, before)
        self.assertTrue(progress)

    def test_range_alignment_and_asymmetric_grid_include_endpoints_and_zero(self):
        settings = SearchSettings(-4.5, 2.1, 2.3, .4).validate()
        self.assertEqual(settings.height_candidates(), [-.004, -.003, -.002, -.001, 0., .001, .002])
        travel = settings.travels()
        self.assertEqual((travel[0], travel[-1]), (-2.3, .4))
        self.assertIn(0., travel)
        self.assertLessEqual(max(np.diff(travel)), 1.)
        for bad in (SearchSettings(1.1, 1.9, 5, 5), SearchSettings(2, 1, 5, 5),
                    SearchSettings(-501, 10, 5, 5), SearchSettings(0, 10, 0, 0),
                    SearchSettings(0, 10, 201, 0), SearchSettings(0, float("nan"), 5, 5),
                    SearchSettings(0, 10, 5, 5, -1)):
            with self.subTest(settings=bad), self.assertRaises(ValueError):
                bad.validate()

    def test_ground_touch_below_ground_and_off_diagonal_poles_are_rejected(self):
        # No sampled denominator is zero. Opposite signs still prove a pole
        # lies between samples; checking only equal-wheel heave is insufficient.
        pole = check_travel(SyntheticMotion(lambda travel: -.2 if travel < 0 else .1), [-1., 1.])
        self.assertFalse(pole.valid)
        self.assertIn("奇異", pole.reason)
        self.assertFalse(check_travel(SyntheticMotion(lambda travel: -.05), [0.]).valid)
        self.assertFalse(check_travel(SyntheticMotion(lambda travel: 0.), [0.]).valid)
        self.assertTrue(check_travel(SyntheticMotion(), [-1., 0., 1.]).valid)
        self.assertFalse(check_travel(SyntheticMotion(), [0.], clearance_mm=30.).valid)

    def test_unreachable_stroke_is_rejected_instead_of_skipped(self):
        def side(travel):
            if travel > 0:
                raise Unreachable("not reachable")
            return SyntheticMotion().side(travel)
        self.assertFalse(check_travel(SimpleNamespace(side=side), [-1., 0., 1.]).valid)

    def test_no_solution_cancellation_and_hardpoint_mode_preserve_design(self):
        project = default_project()
        before = copy.deepcopy(project)
        result = search_roll_center(project, "front", SearchSettings(-5, 0, 25, 25))
        self.assertIsNone(result.project)
        self.assertEqual(result.tested, 6)
        self.assertTrue(result.rejected)
        with self.assertRaises(SearchCancelled):
            search_roll_center(project, "front", SearchSettings(-5, 100, 25, 25), cancelled=lambda: True)
        with self.assertRaises(SearchCancelled):
            check_travel(SyntheticMotion(), [0.], cancelled=lambda: True)
        self.assertEqual(project, before)
        project["hardpoints"] = {"enabled": True, "corners": hardpoints.from_scene(Engine(project).scene)}
        with self.assertRaisesRegex(ValueError, "硬點"):
            search_roll_center(project, "front", SearchSettings(-5, 100, 25, 25))

    def test_manual_mount_fallbacks_are_not_accepted(self):
        project = default_project()
        project["arm_mount_x"]["front"]["upper_front_x"] = .8
        result = search_roll_center(project, "front", SearchSettings(34, 35, 25, 25))
        self.assertIsNone(result.project)
        self.assertTrue(any("A 臂" in reason for reason in result.rejected))


@unittest.skipUnless(os.environ.get("SUSPENSION_GEOMETRY_LAB_GUI_TEST") == "1", "Opt-in native window tests")
class RollCenterSearchGuiTests(unittest.TestCase):
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

    def test_buttons_and_single_lower_bound_input_use_current_drafts(self):
        for axle in ("front", "rear"):
            panel = self.app.panels[axle]
            entry = panel.entry_widgets[(axle, "rc_height")]
            self.assertEqual(str(entry.master), str(entry.action_button.master))
            self.assertEqual(entry.action_button.cget("text"), "尋找最佳roll center")
            panel.entries[("analysis", "bump_mm")].set("20")
            with patch("suspension_geometry_lab.gui.simpledialog.askfloat", return_value=-.005) as ask, \
                    patch("suspension_geometry_lab.gui.RollCenterSearchDialog") as dialog:
                entry.action_button.invoke()
                self.assertEqual(ask.call_args.kwargs["initialvalue"], -.005)
                owner, project, settings = dialog.call_args.args
                self.assertIs(owner, panel)
                self.assertEqual(settings.height_min_mm, -5)
                self.assertEqual(settings.bump_mm, 20)
                self.assertEqual(project["analysis"]["bump_mm"], 20)
            panel.rc_search_dialog = None
        with patch("suspension_geometry_lab.gui.simpledialog.askfloat", return_value=None), \
                patch("suspension_geometry_lab.gui.RollCenterSearchDialog") as dialog:
            self.app.panels["front"].find_roll_center()
            dialog.assert_not_called()

    def test_real_search_applies_best_height_and_marks_project_dirty(self):
        panel = self.app.panels["front"]
        with patch("suspension_geometry_lab.gui.simpledialog.askfloat", return_value=-.005):
            panel.find_roll_center()
        dialog = panel.rc_search_dialog
        end = time.monotonic()+15
        while dialog.poll_timer is not None and time.monotonic() < end:
            self.app.update()
            time.sleep(.01)
        self.assertIsNone(dialog.poll_timer)
        self.assertEqual(self.app.project["front"]["rc_height"], .034)
        self.assertEqual(panel.entries[("front", "rc_height")].get(), "0.034")
        self.assertTrue(self.app.dirty)
        self.assertIn("已套用", dialog.status.get())
        dialog.destroy()

    def test_stale_results_and_cancel_never_replace_the_current_project(self):
        from suspension_geometry_lab.roll_center_dialog import RollCenterSearchDialog
        panel = self.app.panels["front"]
        before = copy.deepcopy(self.app.project)
        settings = SearchSettings(34, 34, 25, 25)
        result = search_roll_center(before, "front", settings)
        dialog = RollCenterSearchDialog(panel, before, settings)
        self.app.revision += 1
        dialog.finish(result)
        self.assertIn("專案已變更", dialog.status.get())
        dialog.destroy()
        self.assertTrue(dialog.cancel_event.is_set())
        self.assertEqual(self.app.project, before)
        points = hardpoints.from_scene(self.app.engine.scene)
        project = copy.deepcopy(before)
        project["hardpoints"] = {"enabled": True, "corners": points}
        self.app.apply_project(project)
        for axle in ("front", "rear"):
            self.assertTrue(self.app.panels[axle].entry_widgets[(axle, "rc_height")].action_button.instate(["disabled"]))


if __name__ == "__main__":
    unittest.main()
