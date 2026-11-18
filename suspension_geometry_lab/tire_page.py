"""Native Pacejka tire response and kingpin torque tab."""

from pathlib import Path
import tkinter as tk
from tkinter import filedialog, ttk

import numpy as np

from .charts import LabChart
from .tire_preview import CORNER_LABELS, TirePreview, wheel_preview_pose
from .tire_response import (CORNERS, DEFAULT_TIR, FRONT_CORNERS, TIR_DIR, calculate_front_axle_response,
                            calculate_tire_response, load_tire_parameters)
from .widgets import ScrollFrame


INPUTS = (
    ("steer_deg", "主銷轉向角 (°)", "0"),
    ("slip_angle_deg", "輪胎側偏角 α (°)", "0"),
    ("slip_ratio", "縱向滑移率 κ (−)", "0"),
    ("wheel_camber_deg", "Camber 外傾角 (°)", ""),
    ("fz_n", "單輪輪荷 (N)", ""),
    ("pressure_kpa", "胎壓 (kPa)", ""),
    ("speed_mps", "前進速度 (m/s)", ""),
)
FRONT_AXLE = "前軸 FL + FR"
ORIENTATION_KEYS = ("steer_deg", "slip_angle_deg", "wheel_camber_deg")
FRONT_ALIGNING_PLOT = "前軸回正力矩（含 Fy）"
FRONT_KINGPIN_PLOT = "前軸主銷力矩"
AXLE_NOTE = ("Δα 同時加到左右輪的輸入側偏角，0° 為目前工況。回正力矩沿路面向上為正，"
             "主銷力矩沿各自下球接點指向上球接點為正。")
PEAK_DIRECTIONS = ((1, "正向峰值", 24), (-1, "負向峰值", -50))
RESULTS = {
    "torque_total_nm": "主銷合成力矩 (N·m)",
    "aligning_total_nm": "回正合成力矩 (N·m)",
    "mz_nm": "　輪胎回正力矩 Mz (N·m)",
    "trail_moment_nm": "　Fy 經機械拖距的力矩 (N·m)",
    "torque_mz_nm": "Mz 在主銷上的分量 (N·m)",
    "torque_contact_nm": "接地力在主銷上的合計 (N·m)",
    "torque_fx_nm": "　Fx 造成的主銷力矩 (N·m)",
    "torque_fy_nm": "　Fy 造成的主銷力矩 (N·m)",
    "torque_fz_nm": "　Fz 造成的主銷力矩 (N·m)",
    "fx_n": "縱向力 Fx (N)",
    "fy_n": "側向力 Fy (N)",
    "fz_n": "輪荷 Fz (N)",
    "wheel_camber_deg": "Camber 外傾角 (°)",
    "model_slip_angle_deg": "送入 TIR 的側偏角 (°)",
    "model_camber_deg": "送入 TIR 的 IA (°)",
    "pressure_kpa": "採用的胎壓 (kPa)",
    "speed_mps": "採用的前進速度 (m/s)",
}
PLOTS = {
    "回正與主銷力矩": ("N·m", (
        ("torque_total_nm", "主銷合成力矩", "#38bdf8"),
        ("mz_nm", "輪胎 Mz", "#34d399"),
        ("torque_mz_nm", "Mz 投影至主銷", "#a78bfa"),
        ("torque_contact_nm", "接地力 × 力臂", "#fbbf24"),
    )),
    "輪胎接地力": ("N", (("fy_n", "Fy 側向力", "#38bdf8"), ("fx_n", "Fx 縱向力", "#34d399"))),
}
AXLE_PLOTS = {
    title: ("N·m", (
        (f"front_{key}", "前軸合計", "#38bdf8"),
        (f"fl_{key}", "左前輪 FL", "#34d399"),
        (f"fr_{key}", "右前輪 FR", "#fbbf24"),
    ))
    for title, key in ((FRONT_ALIGNING_PLOT, "aligning_total_nm"), (FRONT_KINGPIN_PLOT, "torque_total_nm"))
}


class TirePanel(ttk.Frame):
    key = "tire"
    plot_key = "slip_angle"

    def __init__(self, app):
        super().__init__(app.notebook, padding=8)
        self.app = app
        self.state = self.mapping = None
        self.ready_for_export = False
        self.sources = {path.name: path for path in sorted(TIR_DIR.glob("*.tir"))}
        self.source = tk.StringVar(value=DEFAULT_TIR)
        self.corner = tk.StringVar(value=FRONT_AXLE)
        self.plot = tk.StringVar()
        self.notice = tk.StringVar()
        self.source_side = tk.StringVar()
        self.visible_corners = ()
        self.preview_job = None

        left = ttk.Frame(self, width=365)
        left.pack(side="left", fill="y", padx=(0, 12))
        left.pack_propagate(False)
        scroll = ScrollFrame(left)
        scroll.pack(fill="both", expand=True)
        form = scroll.content
        ttk.Label(form, text="MF6.1.2 輪胎參數", font=("Microsoft JhengHei UI", 11, "bold")).pack(anchor="w")
        self.selector = ttk.Combobox(form, state="readonly", textvariable=self.source, values=tuple(self.sources))
        self.selector.pack(fill="x", pady=(5, 6))
        self.selector.bind("<<ComboboxSelected>>", lambda _: self.request())
        ttk.Button(form, text="匯入 .tir", command=self.import_tir).pack(anchor="w")
        ttk.Label(form, textvariable=self.source_side, foreground="#475569").pack(anchor="w", pady=(4, 0))

        ttk.Label(form, text="輪胎工況", font=("Microsoft JhengHei UI", 11, "bold")).pack(anchor="w", pady=(12, 4))
        row = ttk.Frame(form)
        row.pack(fill="x", padx=(0, 9))
        ttk.Label(row, text="分析輪位").pack(side="left")
        corner_select = ttk.Combobox(row, state="readonly", textvariable=self.corner,
                                     values=(FRONT_AXLE, *CORNERS), width=18)
        corner_select.pack(side="right")
        corner_select.bind("<<ComboboxSelected>>", lambda _: self.request())

        fields = ttk.Frame(form)
        fields.pack(fill="x", pady=(8, 0), padx=(0, 5))
        fields.columnconfigure(0, weight=1)
        for index, (_, label, _) in enumerate(INPUTS, start=1):
            ttk.Label(fields, text=label).grid(row=index, column=0, sticky="w", pady=3)
        self.inputs = {}
        self.wheel_fields = {}
        for corner in CORNERS:
            self.inputs[corner] = {}
            widgets = self.wheel_fields[corner] = [ttk.Label(fields, text=CORNER_LABELS[corner])]
            for key, _, default in INPUTS:
                variable = self.inputs[corner][key] = tk.StringVar(value=default)
                variable.trace_add("write", lambda *_, key=key: self.input_changed(key))
                entry = ttk.Entry(fields, textvariable=variable, width=9)
                entry.bind("<Return>", lambda _: self.request())
                widgets.append(entry)
        ttk.Label(form, text="Camber 負值輪頂內傾，正值外傾，留白採轉向後的幾何值。\n"
                             "兩輪側偏角皆以行進方向偏向車輪左側為正。\n"
                             "輪荷留白採靜態單輪荷重。胎壓與速度留白採 TIR 標稱值。",
                  foreground="#475569", wraplength=310).pack(fill="x", pady=(8, 4))
        ttk.Button(left, text="計算輪胎力與力矩", command=self.request).pack(fill="x", pady=(6, 0))

        right = ttk.Frame(self)
        right.pack(side="left", fill="both", expand=True)
        status = ttk.Label(right, textvariable=self.notice, foreground="#92400e", wraplength=650)
        status.pack(fill="x")
        status.bind("<Configure>", lambda event: status.configure(wraplength=max(300, event.width - 8)))

        self.plot_note = tk.StringVar()
        note = ttk.Label(right, textvariable=self.plot_note, wraplength=650, foreground="#475569", padding=(0, 5))
        note.pack(side="bottom", fill="x")
        note.bind("<Configure>", lambda event: note.configure(wraplength=max(300, event.width - 8)))
        table = ttk.Frame(right)
        table.pack(side="bottom", fill="x")
        self.metrics = ttk.Treeview(table, columns=("metric", "first", "right", "front"), show="headings", height=7)
        self.metrics.heading("metric", text="目前工況")
        self.metrics.heading("right", text=CORNER_LABELS["FR"])
        self.metrics.heading("front", text="前軸合計")
        self.metrics.column("metric", width=215, minwidth=205)
        for col in ("first", "right", "front"):
            self.metrics.column(col, width=126, anchor="e", stretch=False)
        self.metrics.tag_configure("total", background="#dbeafe")
        self.metrics.tag_configure("peak", background="#ecfeff")
        self.metrics.pack(side="left", fill="both", expand=True)
        scrollbar = ttk.Scrollbar(table, command=self.metrics.yview)
        scrollbar.pack(side="right", fill="y")
        self.metrics.configure(yscrollcommand=scrollbar.set)
        display = ttk.Frame(right)
        display.pack(fill="both", expand=True, pady=(4, 8))
        self.preview = TirePreview(display)
        self.preview.pack(side="left", anchor="n", padx=(0, 8))
        curve = ttk.Frame(display)
        curve.pack(fill="both", expand=True)
        toolbar = ttk.Frame(curve, padding=(0, 0, 0, 5))
        toolbar.pack(fill="x")
        ttk.Label(toolbar, text="圖表").pack(side="left")
        self.plot_selector = ttk.Combobox(toolbar, state="readonly", textvariable=self.plot, width=20)
        self.plot_selector.pack(side="left", padx=8)
        self.plot_selector.bind("<<ComboboxSelected>>", lambda _: self.draw_curve())
        ttk.Button(toolbar, text="重設視角", command=lambda: self.chart.reset()).pack(side="left")
        self.chart = LabChart(curve)
        self.chart.pack(fill="both", expand=True)
        self.configure_mode()

    def configure_mode(self):
        selection = self.corner.get()
        is_axle = selection == FRONT_AXLE
        corners = FRONT_CORNERS if is_axle else (selection,)
        if corners == self.visible_corners:
            return
        for corner in self.visible_corners:
            for widget in self.wheel_fields[corner]:
                widget.grid_remove()
        for col, corner in enumerate(corners, start=1):
            for row, widget in enumerate(self.wheel_fields[corner]):
                widget.grid(row=row, column=col, padx=(3, 0), pady=2)
        self.visible_corners = corners
        self.plots = AXLE_PLOTS if is_axle else PLOTS
        self.plot_selector.configure(values=tuple(self.plots))
        if self.plot.get() not in self.plots:
            self.plot.set(next(iter(self.plots)))
        self.metrics.heading("first", text=CORNER_LABELS[corners[0]])
        self.metrics.configure(displaycolumns=("metric", "first", "right", "front") if is_axle else ("metric", "first"))
        self.plot_note.set(AXLE_NOTE if is_axle else
                           "轉向採 WE3D 靜態輪端幾何。力矩正軸沿路面向上（Mz）或下球接點指向上球接點（主銷）。")

    def input_changed(self, key):
        self.ready_for_export = False
        self.notice.set("工況已變更，按「計算輪胎力與力矩」更新下方結果。")
        if key in ORIENTATION_KEYS and self.preview_job is None:
            self.preview_job = self.after_idle(self.update_preview)

    def update_preview(self):
        self.preview_job = None
        wheels = {}
        for corner in self.visible_corners:
            try:
                inputs = self.read_inputs(corner, ORIENTATION_KEYS)
                wheels[corner] = wheel_preview_pose(self.app.engine, corner, **inputs)
            except (ValueError, ArithmeticError) as exc:
                wheels[corner] = str(exc)
        self.preview.set(wheels)

    def import_tir(self):
        selected = filedialog.askopenfilename(parent=self, title="匯入 MF6.1 輪胎參數",
                                               initialdir=TIR_DIR, filetypes=[("TIR", "*.tir")])
        if not selected:
            return
        path = Path(selected)
        try:
            params = load_tire_parameters(path)
        except (OSError, ValueError) as exc:
            self.notice.set(f"無法匯入 {path.name}：{exc}")
            return
        label = f"匯入 · {path.name}"
        self.sources[label] = path
        self.selector.configure(values=tuple(self.sources))
        self.source.set(label)
        self.request(params)

    def read_inputs(self, corner, keys=None):
        inputs = {}
        for key, label, default in INPUTS:
            if keys is not None and key not in keys:
                continue
            raw = self.inputs[corner][key].get().strip()
            try:
                inputs[key] = float(raw) if raw or default else None
            except ValueError as exc:
                raise ValueError(f"{corner} 的{label}需要數值。") from exc
        if not -90 < inputs["slip_angle_deg"] < 90:
            raise ValueError(f"{corner} 的側偏角必須介於 −90° 與 90° 之間。")
        return inputs

    def request(self, params=None):
        self.configure_mode()
        if self.preview_job is not None:
            self.after_cancel(self.preview_job)
        self.update_preview()
        self.ready_for_export = False
        self.metrics.delete(*self.metrics.get_children())
        self.state = self.mapping = None
        is_axle = self.corner.get() == FRONT_AXLE
        try:
            source = self.sources.get(self.source.get())
            if source is None:
                raise ValueError("找不到所選輪胎的 TIR 參數。")
            params = load_tire_parameters(source) if params is None else params
            side = str(params.get("TYRESIDE", "left")).strip().lower()
            self.source_side.set({"left": "左側 TIR · 右輪自動鏡射", "right": "右側 TIR · 左輪自動鏡射",
                                  "symmetric": "對稱 TIR"}[side])
            if is_axle:
                left, right = (self.read_inputs(corner) for corner in FRONT_CORNERS)
                alphas = np.array([left["slip_angle_deg"], right["slip_angle_deg"]])
                lo = max(-12., -90. - alphas.min())
                hi = min(12., 90. - alphas.max())
                samples = np.unique(np.append(np.linspace(lo, hi, 121), 0.))
                samples = samples[np.all(np.abs(alphas[:, None] + samples) < 90., axis=0)]
                rows, warnings = calculate_front_axle_response(self.app.engine, params, left=left, right=right,
                                                                slip_offsets_deg=samples)
                position = 0.
            else:
                corner = self.corner.get()
                inputs = self.read_inputs(corner)
                position = inputs.pop("slip_angle_deg")
                span = max(12., abs(position))
                samples = np.unique(np.append(np.linspace(-span, span, 121), position))
                rows, warnings = calculate_tire_response(self.app.engine, params, corner=corner,
                                                          slip_angles_deg=samples, **inputs)
        except (OSError, ValueError, ArithmeticError) as exc:
            self.notice.set(str(exc))
            self.chart.set(None)
            return
        for row in rows:
            row["tir_file"] = source.name
        current = rows[int(np.searchsorted(samples, position))]
        self.state = {"metrics": current}
        self.mapping = {"rows": rows}
        self.ready_for_export = True
        self.notice.set("　".join(warnings))
        prefixes = ("fl_", "fr_", "front_") if is_axle else ("",)
        for key, label in RESULTS.items():
            tags = ("total",) if key in ("aligning_total_nm", "torque_total_nm") else ()
            values = [f"{current[prefix + key]:.5g}" if prefix + key in current else "—" for prefix in prefixes]
            self.metrics.insert("", "end", values=(label, *values), tags=tags)
        self.draw_curve()

    def draw_curve(self):
        if self.mapping is None:
            return
        rows = self.mapping["rows"]
        title = self.plot.get()
        unit, series = self.plots[title]
        is_axle = self.corner.get() == FRONT_AXLE
        x_key = "slip_offset_deg" if is_axle else "slip_angle_deg"
        angle_symbol = "Δα" if is_axle else "α"
        x_label = "側偏角變化 Δα (°)" if is_axle else "輪胎側偏角 α (°)"
        x = [r[x_key] for r in rows]
        current = self.state["metrics"]
        angle, value = current[x_key], current[series[0][0]]
        markers = [{"position": [angle, value], "radius": 5, "name": "目前工況", "color": "#fb7185",
                    "label_lines": [f"{angle_symbol} = {angle:g}°", f"{series[0][1]} {value:.4g} {unit}"]}]
        kingpin = title == FRONT_KINGPIN_PLOT
        peak_key = "front_aligning_total_nm" if is_axle else "mz_nm"
        peak_series = series if kingpin else [curve for curve in series if curve[0] == peak_key]
        for index, (sign, label, y_offset) in enumerate(PEAK_DIRECTIONS):
            iid = f"kingpin-peak-{sign}"
            if self.metrics.exists(iid):
                self.metrics.delete(iid)
            peak_values = {}
            for key, series_label, color in peak_series:
                peak = max(rows, key=lambda row: sign * row[key])
                peak_angle, peak_torque = peak[x_key], peak[key]
                if sign * peak_torque <= 0:
                    continue
                boundary = peak is rows[0] or peak is rows[-1]
                labels = [f"{label} {peak_torque:+.4g} {unit}",
                          f"{angle_symbol} = {peak_angle:g}°" + ("（邊界）" if boundary else "")]
                if kingpin:
                    edge = "*" if boundary else ""
                    peak_values[key] = f"{peak_torque:+.4g} @ {peak_angle:+.4g}°{edge}"
                    labels = [f"{peak_torque:+.4g} {unit}{edge}"]
                markers.append({
                    "position": [peak_angle, peak_torque], "radius": 5,
                    "name": f"{series_label} {label}" if kingpin else label,
                    "color": color, "offset": [12, sign * 18 if kingpin else y_offset], "label_lines": labels,
                    "leader": kingpin,
                })
            if kingpin:
                values = [peak_values.get(f"{prefix}_torque_total_nm", "—") for prefix in ("fl", "fr", "front")]
                self.metrics.insert("", index, iid=iid, values=(f"{label} (N·m @ Δα)", *values), tags=("peak",))
        if is_axle:
            note = " * 表示峰值位於掃描端點。" if kingpin else " 回正力矩包含輪胎 Mz 與 Fy 經機械拖距的力矩。"
            self.plot_note.set(AXLE_NOTE + note)
        if kingpin:
            self.metrics.yview_moveto(0)
        self.chart.set({
            "title": title, "xlabel": x_label, "ylabel": unit,
            "series": [{"name": label, "x": x, "y": [r[key] for r in rows], "color": color, "width": 2}
                       for key, label, color in series],
            "markers": markers,
            "space_callouts": kingpin,
        })
