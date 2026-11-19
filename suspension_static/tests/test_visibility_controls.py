"""Exercise live view switches without rebuilding or moving static geometry."""

import itertools
import unittest
from unittest.mock import patch

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from static.fv import sus_fv_v5 as fv


class VisibilityControlTests(unittest.TestCase):
    def make_figure(self):
        result = fv.create_figure()
        self.addCleanup(plt.close, result[0])
        result[0].visibility_checkboxes.drawon = False
        return result

    def test_all_switch_combinations_preserve_physical_geometry_and_camera(self):
        fig, ax, front, rear = self.make_figure()
        switches = fig.visibility_checkboxes
        self.assertEqual([label.get_text() for label in switches.labels],
                         ["Filled planes", "Front FV", "Rear FV", "SV"])
        views = (front, rear, fig.side_view_plane)
        children = tuple(map(id, ax.get_children()))
        camera = (ax.elev, ax.azim, ax.get_xlim(), ax.get_ylim(), ax.get_zlim())
        vertices = {name: plane["vertices"].copy() for name, plane in fig.arm_planes.items()}
        centers = {corner: wheel["center"].copy()
                   for corner, wheel in fig.wheel_projection_geometry.items()}
        physical = [artist for tire in fig.tire_artists.values()
                    for artist in (tire["tread"], tire["sidewalls"], *tire["outlines"])]
        physical.extend(artist for kingpin in fig.kingpin_artists.values()
                        for artist in kingpin.values())
        for group_name in ("upper_arm_mount_artists", "lower_arm_mount_artists",
                           "rear_upper_arm_mount_artists", "rear_lower_arm_mount_artists"):
            physical.extend(leg["leg"] for arm in getattr(fig, group_name).values()
                            for leg in arm.values())
        physical.extend(edge for arm in fig.arm_plane_artists.values() for edge in arm["edges"])

        with patch.object(fig.canvas, "draw_idle"):
            for state in itertools.product((False, True), repeat=4):
                with self.subTest(state=state):
                    for index, enabled in enumerate(state):
                        if switches.get_status()[index] != enabled:
                            switches.set_active(index)
                    filled, front_on, rear_on, sv_on = state
                    self.assertEqual(fig.show_reference_planes, filled)
                    self.assertEqual(fig.view_visibility,
                                     {"front": front_on, "rear": rear_on, "sv": sv_on})
                    for view, enabled in zip(views, state[1:]):
                        self.assertEqual(view.visible, enabled)
                        self.assertNotIn(view.plane_surface, view.view_artists)
                        self.assertEqual(view.plane_surface.get_visible(), filled and enabled)
                        self.assertTrue(all(artist.get_visible() == enabled
                                            for artist in view.view_artists))
                    for name, arm in fig.arm_plane_artists.items():
                        self.assertIsNotNone(arm["surface"])
                        self.assertEqual(arm["surface"].get_visible(), filled)
                        fv_on = front_on if name.startswith("F") else rear_on
                        for kind, enabled in (("FV_IC", fv_on), ("SV_IC", sv_on)):
                            anchor = arm["anchors"][kind]
                            self.assertIsNotNone(anchor["marker"])
                            self.assertEqual(anchor["marker"].get_visible(), enabled)
                            self.assertEqual(anchor["label"].get_visible(), enabled)
                    self.assertTrue(all(artist.get_visible() for artist in physical))
                    self.assertEqual(tuple(map(id, ax.get_children())), children)
                    self.assertEqual((ax.elev, ax.azim, ax.get_xlim(), ax.get_ylim(), ax.get_zlim()),
                                     camera)
                    for name, expected in vertices.items():
                        np.testing.assert_array_equal(fig.arm_planes[name]["vertices"], expected)
                    for corner, expected in centers.items():
                        np.testing.assert_array_equal(fig.wheel_projection_geometry[corner]["center"],
                                                      expected)
                    legend_labels = {text.get_text() for legend in fig.legends
                                     for text in legend.get_texts()}
                    self.assertEqual("SV geometry" in legend_labels, sv_on)
                    self.assertEqual("A-arm reference planes" in legend_labels, filled)

    def test_initial_flags_and_switch_state_are_local_to_each_figure(self):
        with patch.multiple(fv, SHOW_REFERENCE_PLANES=True, SHOW_FRONT_FV=False,
                            SHOW_REAR_FV=True, SHOW_SV=True):
            first, _, _, _ = self.make_figure()
            second, _, _, _ = self.make_figure()
            initial = (True, False, True, True)
            self.assertEqual(tuple(first.visibility_checkboxes.get_status()), initial)
            self.assertEqual(tuple(second.visibility_checkboxes.get_status()), initial)
            with patch.object(first.canvas, "draw_idle"):
                first.visibility_checkboxes.set_active(1)
                first.visibility_checkboxes.set_active(0)
            self.assertTrue(first.view_visibility["front"])
            self.assertFalse(first.show_reference_planes)
            self.assertEqual(tuple(second.visibility_checkboxes.get_status()), initial)
            self.assertFalse(second.view_visibility["front"])
            self.assertTrue(second.show_reference_planes)
            self.assertEqual((fv.SHOW_REFERENCE_PLANES, fv.SHOW_FRONT_FV,
                              fv.SHOW_REAR_FV, fv.SHOW_SV), initial)


if __name__ == "__main__":
    unittest.main()
