"""STEP solid dimensions/topology plus optional independent CAD read-back."""
import copy
import gc
import math
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
from suspension_geometry_lab.project import default_project, steering_settings
from suspension_geometry_lab.static.fv.wireframe_geometry import project_fv


def read_bodies(data):
    entities = dict(re.findall(r"^(#\d+)=(.+);$", data, re.MULTILINE))
    for value in entities.values():
        if any(ref not in entities for ref in re.findall(r"#\d+", value)):
            raise AssertionError("Dangling STEP reference")
    vectors = {ref: np.array([float(x) for x in match[1].split(",")])
               for ref, value in entities.items()
               if (match := re.fullmatch(r"(?:CARTESIAN_POINT|DIRECTION)\('',\(([^)]+)\)\)", value))}
    bodies = []
    for value in entities.values():
        match = re.fullmatch(r"MANIFOLD_SOLID_BREP\('([^']+)',(#\d+)\)", value)
        if not match:
            continue
        faces = re.findall(r"#\d+", entities[match[2]])
        surfaces, edges = [], {}
        for face in faces:
            bound, surface = re.findall(r"#\d+", entities[face])
            loop, = re.findall(r"#\d+", entities[bound])
            for oriented in re.findall(r"#\d+", entities[loop]):
                edge, = re.findall(r"#\d+", entities[oriented])
                edges.setdefault(edge, []).append(entities[oriented].endswith(",.T.)"))
            surface_value = entities[surface]
            if surface_value.startswith("CYLINDRICAL_SURFACE"):
                placement, = re.findall(r"#\d+", surface_value)
                center, axis, _ = re.findall(r"#\d+", entities[placement])
                radius = float(surface_value.rsplit(",", 1)[1][:-1])
                surfaces.append((vectors[center], vectors[axis], radius))
        if len(faces) != 4 or len(edges) != 6 or any(sorted(pair) != [False, True] for pair in edges.values()):
            raise AssertionError("Cylinder must have a closed, consistently oriented shell")
        bodies.append((match[1], surfaces))
    return bodies


class StepSolidTests(unittest.TestCase):
    def setUp(self):
        project = default_project()
        project["front"]["static_camber_deg"] = -3.
        project["rear"]["static_camber_deg"] = 2.
        self.engine = Engine(project)
        directory = Path(__file__).resolve().parents[1]/".test-tmp"
        directory.mkdir(exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(dir=directory)
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name)/"實體.step"

    def export(self, page, state=None, **options):
        options.setdefault("export_solids", True)
        write_step(self.path, self.engine, page, state, **options)
        data = self.path.read_text(encoding="ascii")
        self.assertIn("SI_UNIT(.MILLI.,.METRE.)", data)
        return data, read_bodies(data)

    def test_static_members_match_hardpoints_with_requested_diameter(self):
        original = copy.deepcopy(self.engine.project)
        points = hardpoints.from_scene(self.engine.scene)
        x, y, z = points["FR"]["UCA_F"]
        hardpoints.set_point(points, "FR", "UCA_F", [x+.01, y, z])
        self.engine.project["hardpoints"] = {"enabled": True, "corners": points}
        self.engine = Engine(self.engine.project)
        data, bodies = self.export("static", tube_diameter_mm=25.4)
        self.assertEqual(len(bodies), 24)
        self.assertNotIn("POLYLINE(", data)
        for name, surfaces in bodies:
            if name.endswith("tire"):
                continue
            corner, rest = name.split(" ", 1)
            a, b = {"upper arm UFI": ("UBJ", "UCA_F"), "upper arm URI": ("UBJ", "UCA_R"),
                    "lower arm LFI": ("LBJ", "LCA_F"), "lower arm LRI": ("LBJ", "LCA_R"),
                    "upright": ("LBJ", "UBJ")}[rest]
            start, end = np.array([points[corner][a], points[corner][b]])*1000
            for center, axis, radius in surfaces:
                np.testing.assert_allclose(center, start, atol=1e-9)
                np.testing.assert_allclose(axis, (end-start)/np.linalg.norm(end-start), atol=1e-12)
                self.assertAlmostEqual(radius, 12.7)
        self.assertEqual(original["front"], self.engine.project["front"])

    def test_tire_cylinder_uses_width_radius_and_optional_mirrored_camber(self):
        _, tilted = self.export("static")
        _, flat = self.export("static", include_static_camber=False)
        for groups, enabled in ((tilted, True), (flat, False)):
            for name, surfaces in groups:
                if not name.endswith("tire"):
                    continue
                corner = name[:2]
                wheel = self.engine.scene.geometry["wheel_projection_geometry"][corner]
                expected_axis = wheel["installation_matrix"][:, 1] if enabled else np.array([0., 1., 0.])
                for center, axis, radius in surfaces:
                    np.testing.assert_allclose(axis, expected_axis, atol=1e-12)
                    np.testing.assert_allclose(center, (wheel["center"]-self.engine.car.tire_width/2*expected_axis)*1000,
                                               atol=1e-9)
                    self.assertAlmostEqual(radius, self.engine.car.free_radius*1000)
        # Only the tire orientation changes when installation camber is disabled.
        tilted_links = [(n, [s[0].tolist()+s[1].tolist()+[s[2]] for s in ss]) for n, ss in tilted if not n.endswith("tire")]
        flat_links = [(n, [s[0].tolist()+s[1].tolist()+[s[2]] for s in ss]) for n, ss in flat if not n.endswith("tire")]
        self.assertEqual(tilted_links, flat_links)

    def test_static_tie_rod_cylinders_follow_the_centered_link_axes(self):
        settings = steering_settings(self.engine.project)
        settings["front"].update(tie_rod_x=.11, tie_rod_y=.54,
                                 rack_x=.065, rack_z=.22, rack_length=.48, rack_travel_mm=11.)
        self.engine = Engine(self.engine.project | {"steering": settings})
        _, bodies = self.export("static", include_tires=False, tube_diameter_mm=14.)
        self.assertEqual(len(bodies), 24)
        rods = {name: surfaces for name, surfaces in bodies if name.endswith(" tie rod")}
        self.assertEqual(set(rods), {f"{corner} tie rod" for corner in ("FL", "FR", "RL", "RR")})
        for corner, (start, end) in self.engine.static_tie_rods():
            for center, axis, radius in rods[f"{corner} tie rod"]:
                np.testing.assert_allclose(center, start*1000, atol=1e-9)
                np.testing.assert_allclose(axis, (end-start)/np.linalg.norm(end-start), atol=1e-12)
                self.assertAlmostEqual(radius, 7.)

    def test_dynamic_pages_have_only_physical_members_and_current_wheel_centers(self):
        for page, count, tires in (("front", 8, 2), ("rear", 8, 2), ("sv", 6, 4)):
            with self.subTest(page=page):
                state = self.engine.solve_sv(13., -6.) if page == "sv" else self.engine.fv[page].solve(15., -8.)
                data, bodies = self.export(page, state)
                self.assertEqual(len(bodies), count)
                self.assertEqual(sum(name.endswith("tire") for name, _ in bodies), tires)
                data_without_references, _ = self.export(page, state, show_construction=False)
                self.assertEqual(data, data_without_references)
                if page in ("front", "rear"):
                    for name, surfaces in bodies:
                        if name.endswith("tire"):
                            suffix = name[1]
                            center, axis, _ = surfaces[0]
                            actual_center = center+self.engine.car.tire_width*500*axis
                            expected = project_fv(state["points"][suffix+"_wheel_center"],
                                                  self.engine.scene.geometry[page].longitudinal_x)*1000
                            np.testing.assert_allclose(actual_center, expected, atol=1e-9)

    def test_default_is_wireframe_and_tires_can_be_omitted_from_solids(self):
        data, bodies = self.export("static", export_solids=False, tube_diameter_mm=float("nan"))
        self.assertFalse(bodies)
        self.assertIn("GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION", data)
        _, bodies = self.export("static", include_tires=False)
        self.assertEqual(len(bodies), 20)
        self.assertFalse(any(name.endswith("tire") for name, _ in bodies))

    def test_invalid_dimensions_or_pose_preserve_destination(self):
        self.path.write_bytes(b"original")
        for diameter in (0., -1., float("nan"), float("inf"), "invalid"):
            with self.subTest(diameter=diameter), self.assertRaises(ValueError):
                write_step(self.path, self.engine, "static", export_solids=True, tube_diameter_mm=diameter)
            self.assertEqual(self.path.read_bytes(), b"original")
        with self.assertRaises(ValueError):
            write_step(self.path, self.engine, "front", export_solids=True)
        self.engine.car.tire_width = float("nan")
        with self.assertRaises(ValueError):
            write_step(self.path, self.engine, "static", export_solids=True)
        self.assertEqual(self.path.read_bytes(), b"original")


@unittest.skipUnless(os.environ.get("SUSPENSION_GEOMETRY_LAB_CAD_TEST") == "1", "Opt-in Open CASCADE validation")
class StepSolidCadTests(unittest.TestCase):
    def test_independent_cad_kernel_reads_closed_solids_and_exact_volumes(self):
        from OCP.BRep import BRep_Tool
        from OCP.BRepCheck import BRepCheck_Analyzer
        from OCP.BRepGProp import BRepGProp
        from OCP.GProp import GProp_GProps
        from OCP.IFSelect import IFSelect_RetDone
        from OCP.STEPControl import STEPControl_Reader
        from OCP.TopAbs import TopAbs_SHELL, TopAbs_SOLID
        from OCP.TopExp import TopExp_Explorer
        project = default_project()
        project["front"]["static_camber_deg"] = -3.
        project["rear"]["static_camber_deg"] = 2.
        engine = Engine(project)
        directory = Path(__file__).resolve().parents[1]/".test-tmp"
        with tempfile.TemporaryDirectory(dir=directory) as temporary:
            path = Path(temporary)/"cad.step"
            for page, count in (("static", 24), ("front", 8), ("rear", 8), ("sv", 6)):
                state = (None if page == "static" else engine.solve_sv(13., -6.) if page == "sv"
                         else engine.fv[page].solve(15., -8.))
                for include_camber in (False, True):
                    with self.subTest(page=page, static_camber=include_camber):
                        write_step(path, engine, page, state, export_solids=True,
                                   tube_diameter_mm=25.4, include_static_camber=include_camber)
                        reader = STEPControl_Reader()
                        self.assertEqual(reader.ReadFile(str(path)), IFSelect_RetDone)
                        self.assertEqual(reader.TransferRoots(), 1)
                        shape = reader.OneShape()
                        self.assertTrue(BRepCheck_Analyzer(shape).IsValid())
                        solids = TopExp_Explorer(shape, TopAbs_SOLID)
                        volumes = []
                        while solids.More():
                            solid = solids.Current()
                            self.assertTrue(BRepCheck_Analyzer(solid).IsValid())
                            shells = TopExp_Explorer(solid, TopAbs_SHELL)
                            self.assertTrue(shells.More())
                            self.assertTrue(BRep_Tool.IsClosed_s(shells.Current()))
                            properties = GProp_GProps()
                            BRepGProp.VolumeProperties_s(solid, properties)
                            self.assertGreater(properties.Mass(), 0)
                            volumes.append(properties.Mass())
                            solids.Next()
                        self.assertEqual(len(volumes), count)
                        # Expected volumes come from source geometry, not the STEP writer.
                        lengths = []
                        if page == "static":
                            for points in hardpoints.from_scene(engine.scene).values():
                                for a, b in (("UBJ", "UCA_F"), ("UBJ", "UCA_R"), ("LBJ", "LCA_F"),
                                             ("LBJ", "LCA_R"), ("LBJ", "UBJ")):
                                    lengths.append(np.linalg.norm(np.array(points[a])-points[b])*1000)
                        else:
                            for line in state["lines"]:
                                if line["name"] in ("upper", "lower", "upright"):
                                    lengths.append(np.linalg.norm(np.array(line["points"])[1]-line["points"][0])*1000)
                        tire_count = 4 if page in ("static", "sv") else 2
                        expected = [math.pi*12.7**2*length for length in lengths]
                        expected += [math.pi*(engine.car.free_radius*1000)**2*engine.car.tire_width*1000]*tire_count
                        np.testing.assert_allclose(sorted(volumes), sorted(expected), rtol=1e-10)


@unittest.skipUnless(os.environ.get("SUSPENSION_GEOMETRY_LAB_GUI_TEST") == "1", "Opt-in native window tests")
class StepSolidGuiTests(unittest.TestCase):
    def setUp(self):
        from suspension_geometry_lab.gui import App
        project = default_project()
        project["analysis"]["samples"] = 5
        self.app = App(project)
        self.app.update()
        panel = self.app.panels["static"]
        self.app.notebook.select(panel)
        self.app.update()
        end = time.monotonic()+20
        while not panel.ready_for_export and time.monotonic() < end:
            self.app.update()
            time.sleep(.01)
        self.assertTrue(panel.ready_for_export)

    def tearDown(self):
        self.app.dirty = False
        self.app.close()
        self.app.worker_thread.join(timeout=5)
        self.app = None
        gc.collect()

    def test_solid_checkbox_enables_diameter_and_export_remembers_options(self):
        directory = Path(__file__).resolve().parents[1]/".test-tmp"
        with tempfile.TemporaryDirectory(dir=directory) as temporary:
            path = Path(temporary)/"solid.step"

            def choose(dialog):
                self.assertFalse(dialog.export_solids.get())
                self.assertTrue(dialog.diameter_entry.instate(["disabled"]))
                self.assertEqual(dialog.tube_diameter_mm.get(), "20")
                dialog.export_solids.set(True)
                dialog.refresh()
                self.assertFalse(dialog.diameter_entry.instate(["disabled"]))
                self.assertIn("圓柱實體", dialog.tire_check.cget("text"))
                dialog.tube_diameter_mm.set("25.4")
                dialog.include_static_camber.set(False)
                dialog.accept()

            with patch.object(self.app, "wait_window", side_effect=choose), \
                    patch("suspension_geometry_lab.gui.filedialog.asksaveasfilename", return_value=str(path)):
                self.app.export("step")
            self.assertEqual(len(read_bodies(path.read_text(encoding="ascii"))), 24)
            self.assertEqual(self.app.step_options["tube_diameter_mm"], 25.4)
            self.assertTrue(self.app.step_options["export_solids"])

            def check_remembered(dialog):
                self.assertTrue(dialog.export_solids.get())
                self.assertEqual(dialog.tube_diameter_mm.get(), "25.4")
                self.assertFalse(dialog.include_static_camber.get())
                dialog.destroy()

            with patch.object(self.app, "wait_window", side_effect=check_remembered), \
                    patch("suspension_geometry_lab.gui.filedialog.asksaveasfilename") as save:
                self.app.export("step")
                save.assert_not_called()

    def test_invalid_diameter_keeps_dialog_open_without_saving(self):
        from suspension_geometry_lab.step_dialog import StepExportDialog
        dialog = StepExportDialog(self.app, {"export_solids": True})
        try:
            for value in ("", "0", "-1", "nan", "inf", "abc"):
                dialog.tube_diameter_mm.set(value)
                with patch("suspension_geometry_lab.step_dialog.messagebox.showerror") as error:
                    dialog.accept()
                error.assert_called_once()
                self.assertTrue(dialog.winfo_exists())
                self.assertIsNone(dialog.options)
            # A disabled diameter draft does not block wireframe export.
            dialog.export_solids.set(False)
            dialog.refresh()
            dialog.accept()
            self.assertFalse(dialog.options["export_solids"])
            self.assertEqual(dialog.options["tube_diameter_mm"], 20.)
        finally:
            if dialog.winfo_exists():
                dialog.destroy()


if __name__ == "__main__":
    unittest.main()
