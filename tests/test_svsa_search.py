"""SVSA threshold selection, real caster poses and transactional native UI."""
import copy
import gc
import os
import threading
import time
import unittest
from unittest.mock import patch

from suspension_geometry_lab import hardpoints
from suspension_geometry_lab.engine import Engine, Unreachable
from suspension_geometry_lab.project import default_project
from suspension_geometry_lab.roll_center_search import SearchCancelled
from suspension_geometry_lab.svsa_search import (
    CasterChange, SVSASearchSettings, _caster_change, _search_axle, search_svsa, svsa_candidates,
)


class SVSASearchTests(unittest.TestCase):
    def test_ranges_start_front_at_one_and_rear_at_shortest_nondegenerate_length(self):
        self.assertEqual(svsa_candidates("front"), [index/2 for index in range(2, 31)])
        self.assertEqual(svsa_candidates("rear"), [index/2 for index in range(1, 31)])
        with self.assertRaises(ValueError):
            svsa_candidates("invalid")

    def test_real_defaults_select_front_two_point_five_and_rear_four_point_five(self):
        project = default_project()
        before = copy.deepcopy(project)
        settings = SVSASearchSettings()
        result = search_svsa(project, settings)
        self.assertIsNotNone(result.project, result.error)
        self.assertEqual(result.project["front"]["svsa"], 2.5)
        self.assertEqual(result.project["rear"]["svsa"], 4.5)
        engine = Engine(result.project)
        zero, target = (engine.solve_sv(value, value)["metrics"] for value in (0., 7.))
        for axle, choice in (("front", result.front.best), ("rear", result.rear.best)):
            self.assertAlmostEqual(choice.zero_deg, zero[f"caster_{axle}_deg"], places=9)
            self.assertAlmostEqual(choice.target_deg, target[f"caster_{axle}_deg"], places=9)
            expected = copy.deepcopy(before[axle])
            expected["svsa"] = choice.svsa_m
            self.assertEqual(result.project[axle], expected)
        self.assertGreaterEqual(result.front.best.absolute_deg, .15)
        self.assertLessEqual(result.rear.best.absolute_deg, .1)
        self.assertLess(_caster_change(project, "front", 3., 7.).absolute_deg, .15)
        self.assertGreater(_caster_change(project, "rear", 4., 7.).absolute_deg, .1)
        self.assertEqual(result.front.stopped_svsa_m, 3.)
        self.assertEqual(project, before)
        self.assertEqual(result.project["analysis"], before["analysis"])
        self.assertEqual(result.project["vehicle"], before["vehicle"])

    def test_each_axle_selects_its_own_bound_at_real_zero_pitch_heave(self):
        project = default_project()
        before = copy.deepcopy(project)
        cases = {
            ("min", "min"): (2.5, 4.),
            ("max", "min"): (3., 4.),
            ("max", "max"): (3., 4.5),
        }
        for (front_bound, rear_bound), (front_length, rear_length) in cases.items():
            with self.subTest(front=front_bound, rear=rear_bound):
                settings = SVSASearchSettings.from_bounds(7., front_bound, .15, rear_bound, .1)
                result = search_svsa(project, settings)
                self.assertIsNotNone(result.project, result.error)
                self.assertEqual(result.project["front"]["svsa"], front_length)
                self.assertEqual(result.project["rear"]["svsa"], rear_length)
        self.assertEqual(project, before)

    def test_independent_evaluation_matches_full_solver_for_positive_and_negative_heave(self):
        project = default_project()
        for axle, length in (("front", 1.), ("front", 2.5), ("rear", .5), ("rear", 4.5)):
            for heave in (-7., 7.):
                with self.subTest(axle=axle, length=length, heave=heave):
                    change = _caster_change(project, axle, length, heave)
                    candidate = copy.deepcopy(project)
                    candidate[axle]["svsa"] = length
                    engine = Engine(candidate)
                    self.assertAlmostEqual(change.zero_deg, engine.solve_sv(0, 0)["metrics"][f"caster_{axle}_deg"], places=8)
                    self.assertAlmostEqual(change.target_deg, engine.solve_sv(heave, heave)["metrics"][f"caster_{axle}_deg"], places=8)

    def test_front_stops_at_first_failure_and_accepts_absolute_equality(self):
        changes = [CasterChange(length, 5., 5.-delta)
                   for length, delta in ((1., .3), (1.5, .2), (2., .15), (2.5, .14), (3., .2))]
        with patch("suspension_geometry_lab.svsa_search._caster_change", side_effect=changes) as evaluate:
            result = _search_axle(default_project(), "front", SVSASearchSettings(), lambda: False, lambda *args: None)
        self.assertEqual(evaluate.call_count, 4)
        self.assertEqual(result.best.svsa_m, 2.)
        self.assertEqual(result.stopped_svsa_m, 2.5)
        self.assertEqual(result.stop_reason, "caster_limit")

    def test_front_geometry_stops_but_rear_skips_invalid_shorter_lengths(self):
        with patch("suspension_geometry_lab.svsa_search._caster_change",
                   side_effect=[CasterChange(1., 0., .2), Unreachable("無解"), CasterChange(2., 0., .2)]) as evaluate:
            front = _search_axle(default_project(), "front", SVSASearchSettings(), lambda: False, lambda *args: None)
        self.assertEqual(evaluate.call_count, 2)
        self.assertEqual(front.best.svsa_m, 1.)
        self.assertEqual(front.stop_reason, "geometry")
        with patch("suspension_geometry_lab.svsa_search._caster_change",
                   side_effect=[Unreachable("無解"), CasterChange(1., 0., -.2), CasterChange(1.5, 0., -.1)]) as evaluate:
            rear = _search_axle(default_project(), "rear", SVSASearchSettings(), lambda: False, lambda *args: None)
        self.assertEqual(evaluate.call_count, 3)
        self.assertEqual(rear.best.svsa_m, 1.5)
        self.assertEqual(rear.geometry_rejected, 1)

    def test_swapped_strategies_follow_the_bound_and_accept_equality(self):
        settings = SVSASearchSettings.from_bounds(7., "max", .15, "min", .1)
        with patch("suspension_geometry_lab.svsa_search._caster_change",
                   side_effect=[Unreachable("無解"), CasterChange(1.5, 0., -.2),
                                CasterChange(2., 0., -.15)]) as evaluate:
            front = _search_axle(default_project(), "front", settings, lambda: False, lambda *args: None)
        self.assertEqual(evaluate.call_count, 3)
        self.assertEqual(front.stop_reason, "found")
        self.assertEqual(front.best.svsa_m, 2.)
        self.assertEqual(front.geometry_rejected, 1)
        with patch("suspension_geometry_lab.svsa_search._caster_change",
                   side_effect=[CasterChange(.5, 0., -.1), CasterChange(1., 0., -.09),
                                CasterChange(1.5, 0., -.2)]) as evaluate:
            rear = _search_axle(default_project(), "rear", settings, lambda: False, lambda *args: None)
        self.assertEqual(evaluate.call_count, 2)
        self.assertEqual(rear.stop_reason, "caster_limit")
        self.assertEqual(rear.best.svsa_m, .5)
        self.assertEqual(rear.stopped_svsa_m, 1.)

    def test_zero_change_limits_reach_front_maximum_and_rear_minimum(self):
        result = search_svsa(default_project(), SVSASearchSettings(0., 0., 0.))
        self.assertIsNotNone(result.project)
        self.assertEqual(result.front.best.svsa_m, 15.)
        self.assertEqual(result.front.stop_reason, "upper_limit")
        self.assertEqual(result.rear.best.svsa_m, .5)

    def test_one_axle_without_solution_never_returns_a_partial_project(self):
        project = default_project()
        before = copy.deepcopy(project)
        for settings in (SVSASearchSettings(7., 1000., .1), SVSASearchSettings(7., .15, 0.)):
            with self.subTest(settings=settings):
                result = search_svsa(project, settings)
                self.assertIsNone(result.project)
                self.assertIn("一起套用", result.error)
                self.assertEqual(project, before)
        project["arm_mount_x"]["front"]["upper_front_x"] = .8
        rejected = search_svsa(project, SVSASearchSettings())
        self.assertIsNone(rejected.project)
        self.assertIn("A 臂", rejected.front.error)

    def test_combined_pose_rechecks_swapped_bound_before_returning_project(self):
        project = default_project()
        before = copy.deepcopy(project)
        settings = SVSASearchSettings.from_bounds(7., "max", .15, "min", .1)

        def optimistic_front(candidate, axle, length, heave):
            if axle == "front":
                return CasterChange(length, 0., .15)
            return _caster_change(candidate, axle, length, heave)

        with patch("suspension_geometry_lab.svsa_search._caster_change", side_effect=optimistic_front):
            result = search_svsa(project, settings)
        self.assertIsNone(result.project)
        self.assertIn("組合驗證失敗", result.error)
        self.assertEqual(project, before)

    def test_each_axle_requires_exactly_one_finite_nonnegative_limit(self):
        for settings in (
            SVSASearchSettings(front_min_change_deg=None),
            SVSASearchSettings(rear_max_change_deg=None),
            SVSASearchSettings(front_max_change_deg=.15),
            SVSASearchSettings(rear_min_change_deg=.1),
            SVSASearchSettings.from_bounds(7., "max", float("inf"), "min", .1),
            SVSASearchSettings.from_bounds(7., "max", .15, "min", -.1),
        ):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                settings.validate(default_project())
        with self.assertRaises(ValueError):
            SVSASearchSettings.from_bounds(7., "both", .15, "min", .1)

    def test_invalid_settings_cancellation_and_hardpoints_preserve_source(self):
        project = default_project()
        before = copy.deepcopy(project)
        for settings in (SVSASearchSettings(float("nan")), SVSASearchSettings(26),
                         SVSASearchSettings(-26), SVSASearchSettings(7., -.1),
                         SVSASearchSettings(7., .15, -.1), SVSASearchSettings(True),
                         SVSASearchSettings(7., float("inf"))):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                search_svsa(project, settings)
        with self.assertRaises(SearchCancelled):
            search_svsa(project, SVSASearchSettings(), cancelled=lambda: True)
        cancelled = threading.Event()
        for phase in ("front", "rear"):
            cancelled.clear()

            def progress(axle, *args):
                if axle == phase:
                    cancelled.set()

            with self.assertRaises(SearchCancelled):
                search_svsa(project, SVSASearchSettings(), cancelled=cancelled.is_set, progress=progress)
        self.assertEqual(project, before)
        project["hardpoints"] = {"enabled": True, "corners": hardpoints.from_scene(Engine(project).scene)}
        with self.assertRaisesRegex(ValueError, "硬點"):
            search_svsa(project, SVSASearchSettings())


@unittest.skipUnless(os.environ.get("SUSPENSION_GEOMETRY_LAB_GUI_TEST") == "1", "Opt-in native window tests")
class SVSASearchGuiTests(unittest.TestCase):
    def setUp(self):
        from suspension_geometry_lab.gui import App
        project = default_project()
        project["analysis"]["samples"] = 5
        self.app = App(project)
        self.app.update()
        self.panel = self.app.panels["sv"]
        self.app.notebook.select(self.panel)
        self.app.update()
        self.wait_panel()

    def wait_panel(self):
        end = time.monotonic()+15
        while not getattr(self.panel, "ready_for_export", False) and time.monotonic() < end:
            self.app.update()
            time.sleep(.01)
        self.assertTrue(self.panel.ready_for_export, self.app.status.get())

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

    def test_buttons_open_three_defaults_and_search_uses_current_drafts(self):
        self.panel.entries[("front", "anti")].set("0.25")
        for axle in ("front", "rear"):
            self.assertEqual(self.panel.entry_widgets[(axle, "svsa")].action_button.cget("text"), "尋找最佳SVSA")

        def choose(dialog):
            self.assertEqual({key: var.get() for key, var in dialog.inputs.items()},
                             {"heave_mm": "7", "front_limit_deg": "0.15", "rear_limit_deg": "0.1"})
            self.assertEqual({axle: var.get() for axle, var in dialog.bounds.items()},
                             {"front": "min", "rear": "max"})
            dialog.inputs["heave_mm"].set("8")
            dialog.accept()

        with patch.object(self.app, "wait_window", side_effect=choose), \
                patch("suspension_geometry_lab.gui.SVSASearchDialog") as search:
            self.panel.entry_widgets[("front", "svsa")].action_button.invoke()
        owner, project, settings = search.call_args.args
        self.assertIs(owner, self.panel)
        self.assertEqual(settings, SVSASearchSettings(8., .15, .1))
        self.assertEqual(project["front"]["anti"], .25)
        self.panel.svsa_search_dialog = None

    def test_real_search_applies_swapped_bounds_and_reopens_with_same_choices(self):
        def choose(dialog):
            dialog.bounds["front"].set("max")
            dialog.bounds["rear"].set("min")
            dialog.accept()

        with patch.object(self.app, "wait_window", side_effect=choose):
            self.panel.find_svsa()
        dialog = self.panel.svsa_search_dialog
        self.wait_dialog(dialog)
        self.wait_panel()
        self.assertIn("已同時套用", dialog.status.get())
        self.assertEqual(dialog.settings.criterion("front"), ("max", .15))
        self.assertEqual(dialog.settings.criterion("rear"), ("min", .1))
        self.assertEqual(self.app.project["front"]["svsa"], 3.)
        self.assertEqual(self.app.project["rear"]["svsa"], 4.)
        dialog.destroy()

        def confirm_previous_choices(dialog):
            self.assertEqual(dialog.bounds["front"].get(), "max")
            self.assertEqual(dialog.bounds["rear"].get(), "min")
            dialog.destroy()

        with patch.object(self.app, "wait_window", side_effect=confirm_previous_choices):
            self.panel.find_svsa()

    def test_settings_validate_without_closing_and_cancel_never_starts_worker(self):
        from suspension_geometry_lab.svsa_dialog import SVSASettingsDialog
        dialog = SVSASettingsDialog(self.panel, self.app.project, SVSASearchSettings())
        for value in ("", "nan", "1000"):
            dialog.inputs["heave_mm"].set(value)
            dialog.accept()
            self.assertTrue(dialog.winfo_exists())
            self.assertIsNone(dialog.settings)
            self.assertTrue(dialog.error.cget("text"))
        dialog.destroy()
        with patch.object(self.app, "wait_window", side_effect=lambda dialog: dialog.destroy()), \
                patch("suspension_geometry_lab.gui.SVSASearchDialog") as search:
            self.panel.find_svsa()
        search.assert_not_called()

        def stale_settings(dialog):
            self.app.revision += 1
            dialog.accept()

        with patch.object(self.app, "wait_window", side_effect=stale_settings), \
                patch("suspension_geometry_lab.gui.SVSASearchDialog") as search, \
                patch("suspension_geometry_lab.gui.messagebox.showerror") as error:
            self.panel.find_svsa()
        search.assert_not_called()
        self.assertIn("專案已變更", error.call_args.args[1])

    def test_real_search_applies_both_and_shows_tested_pose_in_both_control_modes(self):
        for mode in ("前後行程", "Heave / Pitch"):
            self.panel.mode.set(mode)
            self.panel.change_mode()
            self.wait_panel()
            with patch.object(self.app, "wait_window", side_effect=lambda dialog: dialog.accept()):
                self.panel.find_svsa()
            dialog = self.panel.svsa_search_dialog
            self.wait_dialog(dialog)
            self.wait_panel()
            self.assertIn("已同時套用", dialog.status.get())
            self.assertIn("下一級 3 m", dialog.status.get())
            self.assertEqual(self.app.project["front"]["svsa"], 2.5)
            self.assertEqual(self.app.project["rear"]["svsa"], 4.5)
            self.assertEqual(self.panel.entries[("front", "svsa")].get(), "2.5")
            self.assertEqual(self.panel.entries[("rear", "svsa")].get(), "4.5")
            self.assertAlmostEqual(self.panel.state["metrics"]["heave_mm"], 7., places=6)
            self.assertAlmostEqual(self.panel.state["metrics"]["pitch_deg"], 0., places=6)
            self.assertTrue(self.app.dirty)
            dialog.destroy()

    def test_failed_stale_cancelled_results_and_hardpoint_mode_never_apply(self):
        from suspension_geometry_lab.svsa_dialog import SVSASearchDialog
        before = copy.deepcopy(self.app.project)
        good = search_svsa(before, SVSASearchSettings())
        bad = search_svsa(before, SVSASearchSettings(7., 1000., .1))
        with patch.object(SVSASearchDialog, "run_search", return_value=None):
            dialog = SVSASearchDialog(self.panel, before, SVSASearchSettings())
            dialog.finish(bad)
            self.assertIn("未套用", dialog.status.get())
            self.app.revision += 1
            dialog.finish(good)
            self.assertIn("專案已變更", dialog.status.get())
            dialog.destroy()
            self.assertTrue(dialog.cancel_event.is_set())
        self.assertEqual(self.app.project, before)
        before["hardpoints"] = {"enabled": True, "corners": hardpoints.from_scene(self.app.engine.scene)}
        self.app.apply_project(before)
        for axle in ("front", "rear"):
            self.assertTrue(self.panel.entry_widgets[(axle, "svsa")].action_button.instate(["disabled"]))


if __name__ == "__main__":
    unittest.main()
