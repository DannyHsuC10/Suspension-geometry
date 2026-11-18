"""Analytical load-transfer balance and constrained rear RC optimization."""
import copy
import gc
import json
import os
from pathlib import Path
import queue
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from suspension_geometry_lab.engine import Engine
from suspension_geometry_lab.project import default_project, load_project, save_project, validate
from suspension_geometry_lab.project_import import read_import_project
from suspension_geometry_lab.roll_balance import (
    analytic_rear_solution, balance_from_engine, load_transfer_balance, solve_rear_roll_center,
)
from suspension_geometry_lab.roll_center_search import (
    SearchCancelled, SearchSettings, check_travel, search_balanced_rear_roll_center,
)


class RollBalanceTests(unittest.TestCase):
    def test_hand_calculated_split_conserves_moment_and_uses_roll_stiffness_ratio_once(self):
        vehicle = default_project()["vehicle"]
        vehicle.update(wheelbase=2., cg_to_front=.8, cg_height=.5,
                       track_front=1.2, track_rear=1.4, roll_stiffness_ratio=1.5)
        # wf=.6, wr=.4; sf=.6, sr=.4; hRA=.14, elastic lever=.36.
        front = (.6*.1+.6*.36)/1.2
        rear = (.4*.2+.4*.36)/1.4
        balance = load_transfer_balance(vehicle, .1, .2)
        self.assertAlmostEqual(balance.front_transfer_fraction, front/(front+rear))
        self.assertAlmostEqual(balance.front_transfer_per_g, front/.6)
        self.assertAlmostEqual(balance.rear_transfer_per_g, rear/.4)
        self.assertAlmostEqual(front*1.2+rear*1.4, vehicle["cg_height"])
        self.assertAlmostEqual(balance.error, front/(front+rear)-.5)

    def test_document_formula_equalizes_actual_transfers_for_unequal_axle_weights(self):
        vehicle = default_project()["vehicle"]
        vehicle.update(wheelbase=2., cg_to_front=.8, cg_height=.5,
                       track_front=1.2, track_rear=1.4, roll_stiffness_ratio=1.5)
        height = solve_rear_roll_center(vehicle, .1)
        self.assertAlmostEqual(height, 101/260)
        balance = load_transfer_balance(vehicle, .1, height)
        self.assertAlmostEqual(balance.front_weight_fraction, .6)
        self.assertAlmostEqual(balance.front_transfer_fraction, .5)
        # qF*tF+qR*tR=hCG and qF=qR give q=hCG/(tF+tR).
        front = balance.front_transfer_per_g*.6
        rear = balance.rear_transfer_per_g*.4
        self.assertAlmostEqual(front, .5/(1.2+1.4))
        self.assertAlmostEqual(front, rear)
        vehicle.update(track_rear=1.2, roll_stiffness_ratio=1.)
        self.assertAlmostEqual(solve_rear_roll_center(vehicle, .1), .15)

    def test_symmetric_car_is_balanced_and_more_front_roll_stiffness_moves_transfer_forward(self):
        vehicle = default_project()["vehicle"]
        vehicle.update(wheelbase=2., cg_to_front=1., track_front=1.2, track_rear=1.2)
        balanced = load_transfer_balance(vehicle, .04, .04)
        self.assertAlmostEqual(balanced.error, 0.)
        vehicle["roll_stiffness_ratio"] = 2.
        stiff = load_transfer_balance(vehicle, .04, .04)
        self.assertGreater(stiff.front_transfer_fraction, .5)
        self.assertLess(load_transfer_balance(vehicle, .04, .08).front_transfer_fraction, stiff.front_transfer_fraction)
        # Mass, gravity and arbitrary common lateral acceleration cancel out.
        vehicle.update(mass=1000., gravity=9.9)
        self.assertEqual(stiff, load_transfer_balance(vehicle, .04, .04))

    def test_static_camber_uses_actual_contact_frame_rc_and_ground_at_cg(self):
        project = default_project()
        project["front"]["static_camber_deg"] = -5.
        project["rear"]["static_camber_deg"] = 3.
        engine = Engine(project)
        balance = balance_from_engine(engine)
        front, rear = [engine.fv[axle].solve(0., 0., gains=False)["metrics"] for axle in ("front", "rear")]
        self.assertAlmostEqual(balance.front_rc_m, front["rc_y_prime_mm"]/1000)
        self.assertAlmostEqual(balance.rear_rc_m, rear["rc_y_prime_mm"]/1000)
        wf = balance.front_weight_fraction
        self.assertAlmostEqual(balance.cg_height_m, (wf*front["cg_height_mm"]+(1-wf)*rear["cg_height_mm"])/1000)

    def test_analytic_camber_conversion_matches_engine_without_iteration(self):
        for front_camber, rear_camber in ((0., 0.), (-5., 3.), (2., -4.)):
            with self.subTest(front=front_camber, rear=rear_camber):
                project = default_project()
                project["vehicle"]["roll_stiffness_ratio"] = 1.2
                project["front"]["static_camber_deg"] = front_camber
                project["rear"]["static_camber_deg"] = rear_camber
                project["rear"]["fvsa"] = 1.15
                solution = analytic_rear_solution(project)
                project["rear"]["rc_height"] = solution.rear_design_m
                balance = balance_from_engine(Engine(project))
                self.assertAlmostEqual(balance.front_rc_m, solution.front_rc_m, places=12)
                self.assertAlmostEqual(balance.rear_rc_m, solution.rear_rc_m, places=12)
                self.assertAlmostEqual(balance.cg_height_m, solution.cg_height_m, places=12)
                self.assertAlmostEqual(balance.error, 0., places=12)

    def test_old_projects_default_to_equal_roll_stiffness_and_roundtrip_new_ratio(self):
        project = default_project()
        project["vehicle"].pop("roll_stiffness_ratio")
        original = copy.deepcopy(project)
        self.assertEqual(validate(project)["vehicle"]["roll_stiffness_ratio"], 1.)
        self.assertEqual(project, original)
        directory = Path(__file__).resolve().parents[1]/".test-tmp"
        directory.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=directory) as temporary:
            path = Path(temporary)/"project.json"
            path.write_text(json.dumps(project), encoding="utf-8")
            before = path.read_bytes()
            repaired, missing = read_import_project(path)
            self.assertFalse(missing)
            self.assertEqual(repaired["vehicle"]["roll_stiffness_ratio"], 1.)
            self.assertEqual(path.read_bytes(), before)
            repaired["vehicle"]["roll_stiffness_ratio"] = 1.25
            save_project(path, repaired)
            self.assertEqual(load_project(path)["vehicle"]["roll_stiffness_ratio"], 1.25)
        for value in (0., -1., float("nan"), float("inf")):
            project["vehicle"]["roll_stiffness_ratio"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate(project)

    def test_direct_solution_checks_one_rounded_design_without_height_enumeration(self):
        project = default_project()
        project["vehicle"]["roll_stiffness_ratio"] = 1.2
        original = copy.deepcopy(project)
        settings = SearchSettings(-5, 1000, 25, 25)
        with patch.object(SearchSettings, "height_candidates", side_effect=AssertionError("No height scan")), \
                patch("suspension_geometry_lab.roll_center_search.Engine", wraps=Engine) as build:
            result = search_balanced_rear_roll_center(project, settings)
        self.assertEqual(build.call_count, 2)  # Before/after reporting, not candidate enumeration.
        self.assertEqual(result.project["rear"]["rc_height"], .079)
        self.assertAlmostEqual(result.analytic.rear_rc_m, .07937079648011386)
        self.assertEqual(result.rounded_height_m, .079)
        self.assertLess(abs(result.balance.error), abs(result.reference_balance.error))
        self.assertTrue(result.check.valid)
        self.assertTrue(result.front_check.valid)
        self.assertEqual(result.tested, 1)
        self.assertEqual(result.travel_checked, 1)
        self.assertEqual(result.project["front"], original["front"])
        self.assertEqual(project, original)

    def test_failed_stroke_does_not_fall_back_to_another_height(self):
        project = default_project()
        project["vehicle"]["roll_stiffness_ratio"] = .9
        original = copy.deepcopy(project)
        settings = SearchSettings(-5, 1000, 25, 25)
        result = search_balanced_rear_roll_center(project, settings)
        self.assertIsNone(result.project)
        self.assertEqual(result.rounded_height_m, .004)
        self.assertFalse(result.check.valid)
        self.assertEqual(result.travel_checked, 1)
        self.assertIn("後軸全行程", result.failure_reason)
        # A higher feasible height exists, but would not be the analytical solution.
        project["rear"]["rc_height"] = .034
        self.assertTrue(check_travel(Engine(project).fv["rear"], settings.travels()).valid)
        project["rear"]["rc_height"] = original["rear"]["rc_height"]
        self.assertEqual(project, original)

    def test_out_of_bounds_or_rounded_outside_bounds_never_clamps_or_scans(self):
        project = default_project()
        project["vehicle"]["roll_stiffness_ratio"] = 1.2
        for settings, reason in ((SearchSettings(-5, 50, 25, 25), "解析解設計高度"),
                                 (SearchSettings(79.3, 80, 25, 25), "取整後設計高度")):
            with self.subTest(settings=settings), patch("suspension_geometry_lab.roll_center_search.Engine") as build:
                result = search_balanced_rear_roll_center(project, settings)
                self.assertIsNone(result.project)
                self.assertIn(reason, result.failure_reason)
                self.assertIn("超出", result.failure_reason)
                self.assertAlmostEqual(result.analytic.rear_design_m, .07937079648011386)
                build.assert_not_called()

    def test_invalid_front_remains_fixed_and_failure_is_reported(self):
        project = default_project()
        project["vehicle"]["roll_stiffness_ratio"] = 1.2
        project["front"]["rc_height"] = .001
        result = search_balanced_rear_roll_center(project, SearchSettings(-5, 150, 25, 25))
        self.assertIsNone(result.project)
        self.assertIn("固定前軸", result.failure_reason)
        self.assertEqual(project["front"]["rc_height"], .001)

    def test_cancellation_in_both_phases_and_invalid_mounts_never_modify_design(self):
        project = default_project()
        project["vehicle"]["roll_stiffness_ratio"] = 1.2
        original = copy.deepcopy(project)
        settings = SearchSettings(-5, 150, 5, 5)
        cancelled = threading.Event()
        with self.assertRaises(SearchCancelled):
            search_balanced_rear_roll_center(project, settings, cancelled=lambda: True)
        for phase in ("解析解設計高度", "固定前軸全行程檢查", "解析解後軸全行程檢查"):
            cancelled.clear()

            def progress(_, __, ___, reason):
                if phase in reason:
                    cancelled.set()

            with self.assertRaises(SearchCancelled):
                search_balanced_rear_roll_center(project, settings, cancelled=cancelled.is_set, progress=progress)
        self.assertEqual(project, original)
        project["arm_mount_x"]["rear"]["upper_front_x"] = .8
        result = search_balanced_rear_roll_center(project, settings)
        self.assertIsNone(result.project)
        self.assertTrue(any("A 臂" in reason for reason in result.rejected))

    def test_singular_or_nonfinite_formula_inputs_are_rejected(self):
        vehicle = default_project()["vehicle"]
        for ratio in (0., -1., float("nan"), float("inf")):
            vehicle["roll_stiffness_ratio"] = ratio
            with self.subTest(ratio=ratio), self.assertRaises(ValueError):
                solve_rear_roll_center(vehicle, .035)
        vehicle["roll_stiffness_ratio"] = 1.
        with self.assertRaises(ValueError):
            solve_rear_roll_center(vehicle, float("nan"))


@unittest.skipUnless(os.environ.get("SUSPENSION_GEOMETRY_LAB_GUI_TEST") == "1", "Opt-in native window tests")
class RollBalanceGuiTests(unittest.TestCase):
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

    def test_rear_fields_display_weight_split_and_use_current_drafts_in_search(self):
        panel = self.app.panels["rear"]
        self.assertIn("50.99%", panel.balance_context.cget("text"))
        panel.entries[("vehicle", "wheelbase")].set("2")
        panel.entries[("vehicle", "cg_to_front")].set("0.8")
        panel.entries[("vehicle", "roll_stiffness_ratio")].set("1.2")
        self.assertIn("60.00%／40.00%", panel.balance_context.cget("text"))
        with patch("suspension_geometry_lab.gui.simpledialog.askfloat", return_value=-.005) as ask, \
                patch("suspension_geometry_lab.gui.RollCenterSearchDialog") as dialog:
            panel.find_roll_center()
        self.assertIn("載荷轉移", ask.call_args.args[1])
        project = dialog.call_args.args[1]
        self.assertEqual(project["vehicle"]["roll_stiffness_ratio"], 1.2)
        self.assertEqual(project["vehicle"]["wheelbase"], 2.)
        self.assertIn(("vehicle", "track_front"), panel.entries)
        self.assertIn(("vehicle", "track_rear"), panel.entries)
        panel.rc_search_dialog = None

    def test_real_rear_search_applies_balance_result_and_shows_residual(self):
        panel = self.app.panels["rear"]
        panel.entries[("vehicle", "roll_stiffness_ratio")].set("1.2")
        with patch("suspension_geometry_lab.gui.simpledialog.askfloat", return_value=-.005):
            panel.find_roll_center()
        dialog = panel.rc_search_dialog
        end = time.monotonic()+30
        while dialog.poll_timer is not None and time.monotonic() < end:
            self.app.update()
            time.sleep(.01)
        self.assertIsNone(dialog.poll_timer)
        self.assertIn("已套用", dialog.status.get())
        self.assertIn("取整後平衡誤差", dialog.status.get())
        self.assertIn("求解前載荷轉移占比", dialog.status.get())
        self.assertIn("各占 50%", panel.balance_context.cget("text"))
        self.assertIn("解析後軸 RC", dialog.status.get())
        self.assertGreater(self.app.project["rear"]["rc_height"], .034)
        self.assertEqual(self.app.project["front"]["rc_height"], .035)
        self.assertEqual(self.app.project["vehicle"]["roll_stiffness_ratio"], 1.2)
        self.assertTrue(self.app.dirty)
        dialog.destroy()

    def test_rejected_analytical_solution_is_displayed_without_changing_project(self):
        panel = self.app.panels["rear"]
        before = copy.deepcopy(self.app.project)
        panel.entries[("vehicle", "roll_stiffness_ratio")].set("0.9")
        with patch("suspension_geometry_lab.gui.simpledialog.askfloat", return_value=-.005):
            panel.find_roll_center()
        dialog = panel.rc_search_dialog
        end = time.monotonic()+10
        while dialog.poll_timer is not None and time.monotonic() < end:
            self.app.update()
            time.sleep(.01)
        self.assertIsNone(dialog.poll_timer)
        self.assertIn("未套用", dialog.status.get())
        self.assertIn("解析後軸 RC", dialog.status.get())
        self.assertIn("後軸全行程", dialog.status.get())
        self.assertEqual(self.app.project, before)
        self.assertFalse(self.app.dirty)
        dialog.destroy()

    def test_worker_keeps_front_lowest_search_and_routes_rear_to_balance_search(self):
        from suspension_geometry_lab.roll_center_dialog import RollCenterSearchDialog
        for axle, function in (("front", "search_roll_center"), ("rear", "search_balanced_rear_roll_center")):
            messages = queue.Queue()
            with patch("suspension_geometry_lab.roll_center_dialog."+function, return_value="result") as search:
                RollCenterSearchDialog.run_search(default_project(), axle, SearchSettings(0, 10, 5, 5),
                                                 threading.Event(), messages)
            search.assert_called_once()
            self.assertEqual(messages.get_nowait(), ("result", "result"))


if __name__ == "__main__":
    unittest.main()
