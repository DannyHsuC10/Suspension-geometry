"""Wheel orientation sketches for the Pacejka input page."""

from dataclasses import dataclass
import math
import tkinter as tk
from tkinter import ttk

import numpy as np

from .chart_base import BG
from .static.fv.tk_wireframe import project_points
from .tire_response import UP, tire_orientation


CORNER_LABELS = {"FL": "左前輪 FL", "FR": "右前輪 FR", "RL": "左後輪 RL", "RR": "右後輪 RR"}
VIEWS = {"3D": (math.pi + .28, .48), "後視": (math.pi, 0.), "俯視": (math.pi, math.pi / 2)}
FORWARD = np.array([1., 0., 0.])
HEADING_COLOR = "#38bdf8"
TRAVEL_COLOR = "#fbbf24"
TILT_COLOR = "#f9a8d4"
REFERENCE_COLOR = "#94a3b8"
THETA = np.linspace(0., 2 * math.pi, 33)
FONT = "Microsoft JhengHei UI"


@dataclass(frozen=True)
class WheelPreviewPose:
    steer: float
    slip: float
    camber: float
    radius: float
    width: float
    forward: np.ndarray
    travel: np.ndarray
    axle: np.ndarray


def wheel_preview_pose(engine, corner, steer_deg, slip_angle_deg, wheel_camber_deg=None):
    pose, forward, left, ia, camber = tire_orientation(engine, corner, steer_deg, wheel_camber_deg=wheel_camber_deg)
    alpha, gamma = map(math.radians, (slip_angle_deg, ia))
    travel = math.cos(alpha) * forward + math.sin(alpha) * left
    axle = math.cos(gamma) * left + math.sin(gamma) * UP
    return WheelPreviewPose(steer_deg, slip_angle_deg, camber, pose["radius"],
                            engine.car.tire_width, forward, travel, axle)


class TirePreview(ttk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.wheels = {}
        self.view = tk.StringVar(value="3D")
        toolbar = ttk.Frame(self)
        toolbar.pack(fill="x", pady=(0, 4))
        ttk.Label(toolbar, text="車輪姿態", font=(FONT, 11, "bold")).pack(side="left", padx=(0, 10))
        for name in VIEWS:
            ttk.Radiobutton(toolbar, text=name, variable=self.view, value=name,
                            command=self.draw).pack(side="left", padx=(0, 8))
        self.canvas = tk.Canvas(self, bg=BG, height=285, width=330, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _: self.draw())

    def set(self, wheels):
        self.wheels = wheels
        self.draw()

    def draw(self):
        canvas = self.canvas
        canvas.delete("all")
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if not self.wheels or width <= 1 or height <= 1:
            return
        view = self.view.get()
        azimuth, elevation = VIEWS[view]
        card_width = (width - 8 * (len(self.wheels) + 1)) / len(self.wheels)
        card_bottom = height - 46
        details_y = card_bottom - 70
        drawing_top = 37
        drawing_height = max(1, details_y - drawing_top - 5)
        for index, (corner, wheel) in enumerate(self.wheels.items()):
            x = 8 + index * (card_width + 8)
            middle = x + card_width / 2
            canvas.create_rectangle(x, 8, x + card_width, card_bottom, fill="#111e32", outline="#334155")
            canvas.create_text(middle, 23, text=CORNER_LABELS[corner],
                               fill="#f8fafc", font=(FONT, 11, "bold"))
            if isinstance(wheel, str):
                canvas.create_text(x + 12, 62, anchor="nw", text=wheel, width=card_width - 24,
                                   fill="#fbbf24", font=(FONT, 10))
                continue
            center = np.array([middle, drawing_top + drawing_height * .57])
            scale = min(card_width / 4.2, drawing_height / 3.2, 37) / max(wheel.radius, wheel.width / 2)
            self.draw_wheel(wheel, view, azimuth, elevation, center, scale)
            camber = round(wheel.camber, 2) or 0.
            if camber == 0.:
                tilt = "輪頂直立"
            else:
                tilt = "輪頂向內傾" if camber < 0. else "輪頂向外傾"
            values = (
                (f"主銷轉向 {wheel.steer:+.2f}°", "#e2e8f0"),
                (f"側偏 α {wheel.slip:+.2f}°", TRAVEL_COLOR),
                (f"Camber {camber:+.2f}°", TILT_COLOR),
                (tilt, "#cbd5e1"),
            )
            for i, (value, color) in enumerate(values):
                canvas.create_text(middle, details_y + 19 * i,
                                   text=value, fill=color, font=(FONT, 10))

        legend = []
        if view != "後視":
            legend.extend((("車輪朝向", HEADING_COLOR), ("行進方向", TRAVEL_COLOR)))
        if view != "俯視":
            legend.append(("輪胎中線", TILT_COLOR))
        reference = {"3D": "車身前向／鉛直", "俯視": "車身前向", "後視": "鉛直"}[view]
        legend.append((reference, REFERENCE_COLOR))
        for i, (label, color) in enumerate(legend):
            x, y = 12 + (i % 2) * width / 2, height - 32 + (i // 2) * 18
            canvas.create_line(x, y, x + 17, y, fill=color, width=2,
                               dash=(4, 3) if color == REFERENCE_COLOR else ())
            canvas.create_text(x + 23, y, text=label, anchor="w", fill="#cbd5e1", font=(FONT, 9))

    def draw_wheel(self, wheel, view, azimuth, elevation, center, scale):
        canvas = self.canvas
        radius, half_width = wheel.radius, wheel.width / 2
        radial_up = np.cross(wheel.forward, wheel.axle)
        circle = np.outer(np.sin(THETA), wheel.forward) + np.outer(np.cos(THETA), radial_up)
        rings = [side * half_width * wheel.axle + fraction * radius * circle
                 for side, fraction in ((-1, .88), (-.85, 1), (.85, 1), (1, .88))]
        faces = [(rings[0], "#334155", "#64748b"), (rings[-1], "#334155", "#64748b")]
        for inside, outside, color in zip(rings, rings[1:], ("#56677f", "#36465d", "#56677f")):
            for i in range(len(THETA) - 1):
                faces.append((np.array([inside[i], inside[i + 1], outside[i + 1], outside[i]]), color, "#475569"))

        camera = np.array([math.cos(azimuth) * math.cos(elevation),
                           math.sin(azimuth) * math.cos(elevation), math.sin(elevation)])

        def screen(points):
            return (center + project_points(points, azimuth, elevation) * [scale, -scale]).ravel().tolist()

        for points, color, outline in sorted(faces, key=lambda face: np.mean(face[0] @ camera)):
            canvas.create_polygon(*screen(points), fill=color, outline=outline)
        if view != "俯視":
            canvas.create_line(*screen([-1.23 * radius * UP, 1.23 * radius * UP]),
                               fill=REFERENCE_COLOR, dash=(4, 3), width=2)
            canvas.create_line(*screen([-1.09 * radius * radial_up, 1.09 * radius * radial_up]),
                               fill=TILT_COLOR, width=2)
        if view != "後視":
            # Separate arrow lengths keep both tips visible when alpha is zero.
            tail = -.58 * radius * FORWARD
            for direction, length, color in ((FORWARD, 2.3, REFERENCE_COLOR),
                                              (wheel.travel, 2.1, TRAVEL_COLOR),
                                              (wheel.forward, 1.65, HEADING_COLOR)):
                canvas.create_line(*screen([tail, tail + length * radius * direction]), fill=color,
                                   arrow=tk.LAST, arrowshape=(7, 9, 3), width=2,
                                   dash=(4, 3) if color == REFERENCE_COLOR else ())
