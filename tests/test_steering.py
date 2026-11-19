"""Check the TV rack closure, wheel headings and Ackermann convention."""

import math
import unittest

import numpy as np

from suspension_geometry_lab import hardpoints
from suspension_geometry_lab.analysis import make_plot, static_state
from suspension_geometry_lab.engine import Engine, Unreachable
from suspension_geometry_lab.project import RACK_LIMIT_LEFT, RACK_LIMIT_RIGHT, default_project, steering_settings
from suspension_geometry_lab.steering import _rack_rotation, _steering_metrics, solve_steering


class SteeringTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        project = default_project()
        project["front"]["static_camber_deg"] = -2.
        cls.engine = Engine(project)
        cls.settings = steering_settings(project)

    def test_neutral_rack_centers_and_links_mirror_by_axle(self):
        for axle, prefix in (("front", "F"), ("rear", "R")):
            with self.subTest(axle=axle):
                settings = self.settings[axle]
                result = solve_steering(self.engine, axle, settings)
                left, right = (result["wheels"][prefix + side] for side in "LR")
                rack_left, rack_right = (result["rack_ends"][prefix + side] for side in "LR")
                np.testing.assert_array_equal(left["tie_point"], right["tie_point"] * [1., -1., 1.])
                np.testing.assert_array_equal(rack_left - rack_right, [0., settings["rack_length"], 0.])
                np.testing.assert_array_equal(result["rack_center"], [settings["rack_x"], 0., settings["rack_z"]])
                self.assertEqual(left["tie_length"], right["tie_length"])
                self.assertEqual(result["metrics"]["angle_difference_deg"], 0.)
                self.assertIsNone(result["metrics"]["ackermann_pct"])

    def test_neutral_tie_rods_share_their_fv_arm_instant_center(self):
        project = default_project()
        corners = hardpoints.from_scene(self.engine.scene)
        for name in ("UCA_F", "UCA_R"):
            x, y, z = corners["FR"][name]
            hardpoints.set_point(corners, "FR", name, [x, y, z + .015])
        project["hardpoints"] = {"enabled": True, "corners": corners}
        changed_engine = Engine(project)
        for engine in (self.engine, changed_engine):
            for axle, settings in self.settings.items():
                with self.subTest(axle=axle, hardpoints=engine is changed_engine):
                    _, neutral = engine.centered_steering_pickups(axle, settings)
                    right_ic = engine.fv[axle].zero["IC"]
                    for side, sign in (("L", 1.), ("R", -1.)):
                        outer, inner = neutral[side]
                        ic = np.array([sign * right_ic[0], right_ic[1]])
                        projected_rod = outer[1:] - inner[1:]
                        self.assertGreater(np.linalg.norm(projected_rod), 0.)
                        self.assertAlmostEqual(np.linalg.det([projected_rod, ic - inner[1:]]), 0., places=13)
                        self.assertEqual(outer[0], settings["tie_rod_x"])
                        self.assertEqual(outer[1], sign * settings["tie_rod_y"])
                        self.assertEqual(inner[2], settings["rack_z"])

        front = self.settings["front"]
        _, baseline = self.engine.centered_steering_pickups("front", front)
        _, changed = changed_engine.centered_steering_pickups("front", front)
        self.assertNotAlmostEqual(baseline["L"][0][2], changed["L"][0][2])
        moved_x = front | {"tie_rod_x": front["tie_rod_x"] - .03}
        _, moved = self.engine.centered_steering_pickups("front", moved_x)
        self.assertEqual(moved["L"][0][2], baseline["L"][0][2])

    def test_persisted_steering_needs_a_unique_fv_projection(self):
        project = default_project()
        project["steering"] = steering_settings(project)
        front = project["steering"]["front"]
        front["tie_rod_y"] = front["rack_length"] / 2.
        with self.assertRaisesRegex(ValueError, "外接點.*Y 座標重合"):
            Engine(project)

        front["tie_rod_y"] = self.settings["front"]["tie_rod_y"]
        project["front"]["fvsa"] = .35  # Static FV IC Y and left rack Y both equal .275 m.
        with self.assertRaisesRegex(ValueError, "齒條端點.*瞬心.*Y 座標重合"):
            Engine(project)

        project = default_project()
        corners = hardpoints.from_scene(self.engine.scene)
        for arm, joint in (("UCA", "UBJ"), ("LCA", "LBJ")):
            for end in "FR":
                name = f"{arm}_{end}"
                x, y, _ = corners["FR"][name]
                hardpoints.set_point(corners, "FR", name, [x, y, corners["FR"][joint][2]])
        project["hardpoints"] = {"enabled": True, "corners": corners}
        self.assertIsNone(Engine(project).fv["front"].zero["IC"])
        project["steering"] = steering_settings(project)
        with self.assertRaisesRegex(ValueError, "前視 A 臂瞬心.*無法定義"):
            Engine(project)

    def test_static_plots_share_applied_neutral_tie_rod_endpoints(self):
        project = default_project()
        settings = project["steering"] = steering_settings(project)
        settings["front"].update(rack_x=-.14, rack_z=.172, rack_travel_mm=14.)
        settings["rear"].update(rack_z=.207, rack_length=.5, enabled=False)
        engine = Engine(project)
        links = dict(engine.static_tie_rods())
        self.assertEqual(set(links), {"FL", "FR", "RL", "RR"})
        for axle in ("front", "rear"):
            neutral = solve_steering(engine, axle, settings[axle], travel_mm=0.)
            for corner, wheel in neutral["wheels"].items():
                np.testing.assert_array_equal(links[corner], [wheel["tie_point"], neutral["rack_ends"][corner]])
        self.assertEqual(static_state(engine)["metrics"]["wireframe_lines"], len(engine.scene.lines) + 4)
        for projection in ("iso", "front", "side", "top"):
            plot = make_plot("static", projection, engine=engine, groups={"front": False, "rear": False, "sv": False},
                             show_static_references=False)
            rods = [line for line in plot["world_lines"] if line["name"] == "Tie rod"]
            self.assertEqual(len(rods), 4)
            self.assertTrue(all(not line["construction"] for line in rods))
            np.testing.assert_array_equal([line["points"] for line in rods], list(links.values()))
        self.assertEqual(list(self.engine.static_tie_rods()), [])

    def test_rack_travel_preserves_rod_length_and_rotates_wheels_about_kingpins(self):
        for axle, prefix in (("front", "F"), ("rear", "R")):
            # Vary height as well as X so the rod's vertical geometry matters.
            settings = dict(self.settings[axle], rack_z=.175, rack_travel_mm=20.)
            if axle == "rear":
                settings["enabled"] = True
            positive = solve_steering(self.engine, axle, settings)
            negative = solve_steering(self.engine, axle, settings, travel_mm=-20.)
            for corner in positive["wheels"]:
                wheel = positive["wheels"][corner]
                reference = positive["wheels_neutral"][corner]
                rack = positive["rack_ends"][corner]
                np.testing.assert_allclose(rack - positive["rack_ends_neutral"][corner], [0., .02, 0.])
                self.assertAlmostEqual(np.linalg.norm(wheel["tie_point"] - rack), wheel["tie_length"], places=12)
                pose = self.engine.wheel_end_pose(wheel["kingpin_deg"], corner)
                origin = wheel["kingpin_origin"]
                np.testing.assert_allclose(wheel["tie_point"], origin + pose["matrix"] @ (reference["tie_point"] - origin))
                np.testing.assert_allclose(wheel["center"], pose["center"])
                self.assertAlmostEqual(wheel["steer_deg"], math.degrees(math.atan2(-pose["normal"][0], pose["normal"][1])))
                self.assertGreater(abs(wheel["steer_deg"] - wheel["kingpin_deg"]), .001)
                other = negative["wheels"][prefix + ("R" if corner.endswith("L") else "L")]
                self.assertAlmostEqual(wheel["steer_deg"], -other["steer_deg"], places=10)
                np.testing.assert_array_equal(wheel["tire_outline"][0], wheel["tire_outline"][-1])
            self.assertAlmostEqual(positive["metrics"]["ackermann_pct"], negative["metrics"]["ackermann_pct"], places=10)

    def test_ackermann_uses_actual_turn_inside_and_reports_reference_outer_angle(self):
        track, wheelbase = 1.25, 1.53
        inner = math.radians(25.)
        outer = math.degrees(math.atan(1. / (1. / math.tan(inner) + track / wheelbase)))
        for axle, corners in (("front", ["FL", "FR"]), ("rear", ["RL", "RR"])):
            for sign in (1., -1.):
                left_inside = (sign > 0.) == (axle == "front")
                left, right = (25., outer) if left_inside else (outer, 25.)
                wheels = {corner: dict(steer_deg=sign * angle, kingpin_deg=sign * angle)
                          for corner, angle in zip(corners, (left, right))}
                result = _steering_metrics(axle, wheels, track, wheelbase, 20.)
                self.assertEqual(result["inner_corner"], corners[0 if left_inside else 1])
                self.assertAlmostEqual(result["ackermann_pct"], 100.)
                self.assertAlmostEqual(result["ideal_outer_deg"], sign * outer)
                self.assertAlmostEqual(result["angle_difference_deg"], 25. - outer)

                for wheel in wheels.values():
                    wheel["steer_deg"] = sign * 25.
                self.assertAlmostEqual(_steering_metrics(axle, wheels, track, wheelbase, 20.)["ackermann_pct"], 0.)
                wheels[corners[1 if left_inside else 0]]["steer_deg"] = sign * 30.
                self.assertLess(_steering_metrics(axle, wheels, track, wheelbase, 20.)["ackermann_pct"], 0.)

    def test_ackermann_retains_the_small_angle_difference_near_neutral(self):
        settings = self.settings["front"]
        near = solve_steering(self.engine, "front", settings, travel_mm=.01)["metrics"]
        tiny = solve_steering(self.engine, "front", settings, travel_mm=1e-6)["metrics"]
        self.assertGreater(tiny["angle_difference_deg"], 0.)
        self.assertAlmostEqual(tiny["ackermann_pct"], near["ackermann_pct"], delta=.01)

    def test_unreachable_command_and_degenerate_upright_are_rejected(self):
        settings = self.settings["front"] | {RACK_LIMIT_LEFT: 100.}
        with self.assertRaisesRegex(Unreachable, "100 mm"):
            solve_steering(self.engine, "front", settings, travel_mm=100.)
        upright = self.engine.scene.geometry["kingpins"]["FL"]["LBJ"]
        settings = self.settings["front"] | {"tie_rod_x": float(upright[0]), "tie_rod_y": float(upright[1])}
        ic_y, ic_z = self.engine.fv["front"].zero["IC"]
        y_rack = settings["rack_length"] / 2.
        settings["rack_z"] = float(ic_z + (upright[2] - ic_z) * (y_rack - ic_y) / (upright[1] - ic_y))
        with self.assertRaisesRegex(Unreachable, "FL"):
            solve_steering(self.engine, "front", settings)

    def test_signed_rack_limits_allow_exact_endpoints_and_reject_excess(self):
        settings = self.settings["front"] | {RACK_LIMIT_LEFT: 17.125, RACK_LIMIT_RIGHT: 6., "rack_travel_mm": 17.125}
        left = solve_steering(self.engine, "front", settings)
        self.assertEqual(left["rack_center"][1], .017125)
        right = solve_steering(self.engine, "front", settings, travel_mm=-6.)
        self.assertEqual(right["rack_center"][1], -.006)
        for travel in (17.126, -6.001):
            with self.subTest(travel=travel), self.assertRaisesRegex(ValueError, r"-6 ～ \+17\.125 mm"):
                solve_steering(self.engine, "front", settings, travel_mm=travel)

    def test_rear_rack_stays_centered_until_enabled(self):
        settings = self.settings["rear"] | {"rack_travel_mm": 12.}
        for override in (None, -5., 1000.):
            with self.subTest(override=override):
                result = solve_steering(self.engine, "rear", settings, travel_mm=override)
                self.assertEqual(result["rack_center"][1], 0.)
                self.assertEqual(result["metrics"]["rack_travel_mm"], 0.)
                for side in ("left", "right"):
                    self.assertEqual(result["metrics"][side + "_steer_deg"], 0.)
        settings.update(enabled=True)
        self.assertNotEqual(solve_steering(self.engine, "rear", settings)["metrics"]["angle_difference_deg"], 0.)
        with self.assertRaisesRegex(ValueError, "超出允許範圍"):
            solve_steering(self.engine, "rear", settings, travel_mm=1000.)

    def test_reachable_endpoint_cannot_jump_a_disconnected_assembly_branch(self):
        radial, tangent = np.array([1., 0., 0.]), np.array([0., 1., 0.])
        offset = np.array([0., .5, 0.])
        # Neutral and target each have the same valid rod length, but the
        # inner rack ball crosses the rotation center at halfway travel.
        self.assertAlmostEqual(np.linalg.norm(offset - radial), np.linalg.norm(offset - tangent - radial))
        with self.assertRaisesRegex(Unreachable, "1000 mm"):
            _rack_rotation("FL", radial, tangent, offset, -1.)
        with self.assertRaisesRegex(Unreachable, "零位"):
            _rack_rotation("FL", radial, tangent, np.array([.5, 0., 0.]), .1)


if __name__ == "__main__":
    unittest.main()
