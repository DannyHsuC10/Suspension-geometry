"""Front and rear steering rack controls and top-view results."""

import math
import tkinter as tk
from tkinter import ttk

from .charts import LabChart
from .model_notes import MODEL_NOTES
from .project import (RACK_LIMIT_FIELDS, STEERING_FIELDS, rack_travel_bounds, steering_settings,
                      validate_steering_settings)
from .steering import solve_steering
from .steering_chart import TV_PLOTS, steering_chart, steering_sweep_chart
from .tv_analysis import calculate_tv_sweep, steering_camber_metrics
from .tv_lap import calculate_tv_lap_overlay
from .widgets import ModelNotes, ScrollFrame, input_range_label


AXLE_LABELS = {"front": "前軸 FL / FR", "rear": "後軸 RL / RR"}
TRAVEL = "rack_travel_mm"
GEOMETRY_GROUPS = (
    ("立柱拉桿接點", ("tie_rod_x", "tie_rod_y")),
    ("齒條（置中時 Y = 0）", ("rack_x", "rack_z", "rack_length")),
)
GEOMETRY_FIELDS = tuple(key for _, keys in GEOMETRY_GROUPS for key in keys)
RESULT_ROWS = (
    ("rack_travel_mm", "Rack 位移 (mm)", False),
    ("steer_deg", "輪胎轉向角 (°)", True),
    ("angle_difference_deg", "左右轉角差 (°)", False),
    ("ackermann_pct", "阿克曼率 (%)", False),
    ("camber_gain_deg_per_mm", "Camber gain (°/mm)", True),
    ("camber_deg", "轉向後 Camber (°)", True),
    ("camber_change_deg", "轉向造成的 Camber 變化 (°)", True),
    ("kingpin_deg", "繞主銷旋轉角 (°)", True),
    ("steering_camber_gain_deg_per_deg", "轉向 Camber gain (°/°)", True),
    ("left_minus_right_deg", "左輪減右輪轉角 (°)", False),
    ("ideal_outer_deg", "理想外輪轉角 (°)", False),
    ("turn", "彎道內／外輪", False),
)


class SteeringPanel(ttk.Frame):
    key = "tv"
    export_kinds = frozenset(("state", "mapping", "png", "svg"))

    def __init__(self, app):
        super().__init__(app.notebook)
        self.app = app
        self.axle = tk.StringVar(value="front")
        self.drafts = {axle: {key: tk.StringVar() for key in STEERING_FIELDS} for axle in AXLE_LABELS}
        self.rear_enabled = tk.BooleanVar(value=False)
        self.sliding_travel = tk.DoubleVar()
        self.notice = tk.StringVar()
        self.tie_rod_z = tk.StringVar(value="—")
        self.plot = tk.StringVar(value=TV_PLOTS[0][1])
        self.plot_notice = tk.StringVar()
        self.timer = None
        self.chart_key = None
        self.ready_for_export = False
        self.state = None
        self.pose = None
        self.configuration = None
        self.mapping = None
        self.token = 0
        self.sweep_cache = None
        self.overlay_cache = None
        self.queued = None
        self.analysis_error = None
        self.pending_export = None

        self.pane = ttk.Panedwindow(self, orient="horizontal")
        self.pane.pack(fill="both", expand=True)
        left = ttk.Frame(self.pane, width=345)
        self.pane.add(left, weight=0)
        output = ttk.Frame(self.pane)
        self.pane.add(output, weight=1)
        self.scroll = ScrollFrame(left)
        self.scroll.pack(fill="both", expand=True, padx=(10, 4))
        form = self.scroll.content

        axle_settings = ttk.Frame(form)
        axle_settings.pack(fill="x")
        self.add_heading(axle_settings, "車軸設定")
        axle_row = ttk.Frame(axle_settings)
        axle_row.pack(fill="x", pady=2)
        for axle, label in AXLE_LABELS.items():
            ttk.Radiobutton(axle_row, text=label, variable=self.axle, value=axle,
                            command=self.change_axle).pack(side="left", padx=(0, 10))
        self.rear_check = ttk.Checkbutton(axle_settings, text="啟用後輪轉向", variable=self.rear_enabled,
                                         command=self.toggle_rear)

        self.entries = {}
        self.range_refresh = {}
        for title, keys in GEOMETRY_GROUPS:
            self.add_heading(form, title)
            for key in keys:
                self.add_entry(form, key)
            if "tie_rod_y" in keys:
                self.setting_entry(form, "左側 Tie rod 求解 Z (m)", textvariable=self.tie_rod_z, state="readonly")
                ttk.Label(form, text="由同側 Rack 端點與 A 臂前視瞬心求得。", foreground="#64748b",
                          wraplength=280).pack(fill="x", pady=(0, 4))

        self.limit_frame = ttk.Frame(form)
        self.add_heading(self.limit_frame, "Rack 行程設定")
        for key in RACK_LIMIT_FIELDS:
            self.add_entry(self.limit_frame, key)
        self.geometry_hint = ttk.Label(form, text="X 向前、Y 向左、Z 向上。前軸 X = 0，後軸 X = −軸距。"
                                       "右接點鏡射左接點 Y。", foreground="#475569", wraplength=295)
        self.geometry_hint.pack(fill="x", pady=(8, 0))
        self.apply_button = ttk.Button(left, text="套用左側參數並重新解算", command=self.apply)
        self.apply_button.pack(fill="x", padx=10, pady=(10, 6))
        notice = ttk.Label(left, textvariable=self.notice, wraplength=295, foreground="#475569")
        notice.pack(fill="x", padx=10, pady=(0, 8))
        left.bind("<Configure>", lambda event: notice.configure(wraplength=max(180, event.width - 20)))

        toolbar = ttk.Frame(output, padding=(8, 10))
        toolbar.pack(fill="x")
        ttk.Label(toolbar, text="圖表").pack(side="left")
        self.plot_selector = ttk.Combobox(toolbar, state="readonly", textvariable=self.plot,
                                          values=[label for _, label in TV_PLOTS], width=40)
        self.plot_selector.pack(side="left", padx=8)
        self.plot_selector.bind("<<ComboboxSelected>>", lambda _: self.render())
        ttk.Button(toolbar, text="重設視角", command=lambda: self.chart.reset()).pack(side="left", padx=4)
        lap_row = ttk.Frame(output, padding=(8, 2))
        lap_row.pack(fill="x")
        ttk.Button(lap_row, text="匯入 LapSim CSV", command=app.open_lap).pack(side="left")
        self.lap_check = ttk.Checkbutton(lap_row, text="顯示 LapSim 轉向工況", variable=app.show_lap,
                                         command=app.toggle_lap)
        self.lap_check.pack(side="left", padx=8)
        ttk.Label(lap_row, textvariable=app.lap_source, foreground="#475569").pack(side="left", padx=8)

        self.travel_frame = ttk.Frame(output, padding=(8, 2))
        ttk.Label(self.travel_frame, text="Rack 位移 (mm)", width=16).pack(side="left")
        ttk.Label(self.travel_frame, text="+Y 左", foreground="#475569").pack(side="left")
        self.scale = ttk.Scale(self.travel_frame, orient="horizontal", from_=0., to=0.,
                               variable=self.sliding_travel, command=self.slider_changed)
        self.scale.pack(side="left", fill="x", expand=True, padx=8)
        ttk.Label(self.travel_frame, text="−Y 右", foreground="#475569").pack(side="left")
        self.entries[TRAVEL] = ttk.Entry(self.travel_frame, width=10)
        self.entries[TRAVEL].pack(side="left", padx=8)
        self.entries[TRAVEL].bind("<Return>", lambda _: self.request())
        self.zero_button = ttk.Button(self.travel_frame, text="回到零姿態", command=self.zero_travel)
        self.zero_button.pack(side="left")
        self.lock_notice = ttk.Label(output, text="後輪轉向關閉，齒條 Y 位移固定為 0 mm。",
                                     foreground="#475569", padding=(8, 2))
        self.plot_note = ttk.Label(output, textvariable=self.plot_notice, foreground="#475569")
        self.plot_note.pack(fill="x", padx=8, pady=(0, 3))
        self.plot_note.bind("<Configure>", lambda event: self.plot_note.configure(wraplength=max(event.width, 1)))
        self.chart = LabChart(output)
        self.model_notes = ModelNotes(output, self.chart, MODEL_NOTES["tv"])
        self.model_notes.pack(side="bottom", fill="x", padx=8, pady=(0, 5))
        results = ttk.LabelFrame(output, text="轉向結果", padding=(10, 4))
        results.pack(side="bottom", fill="x", padx=8, pady=(0, 6))
        self.chart.pack(fill="both", expand=True, padx=8, pady=6)
        table = ttk.Frame(results)
        table.pack(fill="x")
        self.metrics = ttk.Treeview(table, columns=("metric", "left", "right", "axle"), show="headings", height=7)
        self.metrics.heading("metric", text="目前姿態 / 單位")
        self.metrics.heading("axle", text="本軸")
        for column, width in (("metric", 235), ("left", 105), ("right", 105), ("axle", 170)):
            self.metrics.column(column, width=width, anchor="w" if column == "metric" else "e")
        scrollbar = ttk.Scrollbar(table, command=self.metrics.yview)
        scrollbar.pack(side="right", fill="y")
        self.metrics.configure(yscrollcommand=scrollbar.set)
        self.metrics.pack(side="left", fill="x", expand=True)
        for key, label, _ in RESULT_ROWS:
            self.metrics.insert("", "end", iid=key, values=(label, "—", "—", "—"))
        method = ttk.Label(results, text="輪胎朝左為正。Camber gain (°/mm) 固定當前主銷轉角，計算單輪上壓的外傾率。"
                           "轉向 Camber gain (°/°) 固定懸吊高度。", foreground="#475569", padding=(0, 3))
        method.pack(fill="x")
        results.bind("<Configure>", lambda event: method.configure(wraplength=max(event.width - 24, 1)))

        self.sync()
        for axle, variables in self.drafts.items():
            for key, variable in variables.items():
                variable.trace_add("write", lambda *_args, axle=axle, key=key: self.schedule(axle, key))

    @property
    def inputs(self):
        return self.drafts[self.axle.get()]

    @property
    def plot_key(self):
        return next(key for key, title in TV_PLOTS if title == self.plot.get())

    @property
    def travel_enabled(self):
        return self.axle.get() == "front" or self.rear_enabled.get()

    @staticmethod
    def add_heading(parent, title):
        ttk.Label(parent, text=title, font=("Microsoft JhengHei UI", 11, "bold"),
                  padding=(0, 12, 0, 6)).pack(anchor="w")

    @staticmethod
    def setting_entry(parent, label, **options):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text=label, wraplength=215).pack(side="left", fill="x", expand=True)
        entry = ttk.Entry(row, width=12, **options)
        entry.pack(side="right", padx=(6, 0))
        return entry

    def add_entry(self, parent, key):
        label, unit = STEERING_FIELDS[key]
        entry = self.entries[key] = self.setting_entry(parent, f"{label} ({unit})")
        entry.bind("<Return>", lambda _: self.request())
        if key in RACK_LIMIT_FIELDS:
            self.range_refresh[key] = input_range_label(parent, entry, "允許範圍 0 mm 以上",
                                                        lambda value: math.isfinite(value) and value >= 0.)
        elif key in ("tie_rod_y", "rack_length"):
            self.range_refresh[key] = input_range_label(parent, entry, "允許範圍大於 0 m",
                                                        lambda value: math.isfinite(value) and value > 0.)
        else:
            self.range_refresh[key] = input_range_label(parent, entry, "允許範圍為有限數值", math.isfinite)

    def sync(self):
        settings = steering_settings(self.app.project)
        for axle, variables in self.drafts.items():
            for key, variable in variables.items():
                variable.set(str(settings[axle][key]))
        self.rear_enabled.set(settings["rear"]["enabled"])
        self.token += 1
        self.chart_key = self.sweep_cache = self.overlay_cache = self.queued = None
        self.analysis_error = self.pending_export = None
        self.change_axle()

    def change_axle(self):
        for key, entry in self.entries.items():
            entry.configure(textvariable=self.inputs[key])
        for key, refresh in self.range_refresh.items():
            refresh(self.inputs[key].get())
        prefix = "F" if self.axle.get() == "front" else "R"
        self.metrics.heading("left", text=f"左輪 {prefix}L")
        self.metrics.heading("right", text=f"右輪 {prefix}R")
        self.show_travel_controls()
        self.request()

    def show_travel_controls(self):
        if self.axle.get() == "rear":
            self.rear_check.pack(anchor="w", pady=(2, 0))
        else:
            self.rear_check.pack_forget()
        if self.travel_enabled:
            self.lock_notice.pack_forget()
            self.limit_frame.pack(fill="x", before=self.geometry_hint)
            self.travel_frame.pack(fill="x", before=self.plot_note)
        else:
            self.limit_frame.pack_forget()
            self.travel_frame.pack_forget()
            self.lock_notice.pack(fill="x", before=self.plot_note)

    def toggle_rear(self):
        if not self.rear_enabled.get():
            self.drafts["rear"][TRAVEL].set("0")
        self.show_travel_controls()
        self.request()

    def update_slider(self):
        if not self.travel_enabled:
            self.sliding_travel.set(0.)
            self.scale.state(["disabled"])
            return
        try:
            low, high = rack_travel_bounds({key: self.read_entry(key) for key in RACK_LIMIT_FIELDS})
        except ValueError:
            self.scale.state(["disabled"])
            return
        if (self.scale.cget("to"), self.scale.cget("from")) != (low, high):
            self.scale.configure(from_=high, to=low)
        self.scale.state(["disabled" if low == high else "!disabled"])
        try:
            travel = self.read_entry(TRAVEL)
        except ValueError:
            return
        if math.isfinite(travel):
            self.sliding_travel.set(min(high, max(low, travel)))

    def slider_changed(self, value):
        if self.travel_enabled:
            travel = float(value)
            low, high = self.scale.cget("to"), self.scale.cget("from")
            if low < travel < high:
                travel = min(high, max(low, round(travel, 2)))
            self.inputs[TRAVEL].set(str(travel))

    def zero_travel(self):
        self.inputs[TRAVEL].set("0")
        self.request()

    def schedule(self, axle, key):
        if axle != self.axle.get():
            return
        if key in self.range_refresh:
            self.range_refresh[key](self.inputs[key].get())
        self.ready_for_export = False
        if self.timer is None:
            self.timer = self.after(16, self.request)

    def read_entry(self, key):
        label, unit = STEERING_FIELDS[key]
        try:
            return float(self.inputs[key].get())
        except ValueError as exc:
            raise ValueError(f"{label}需要有效數值（{unit}）。") from exc

    def read_settings(self):
        settings = steering_settings(self.app.project)
        axle = self.axle.get()
        values = settings[axle]
        for key in GEOMETRY_FIELDS:
            values[key] = self.read_entry(key)
        if axle == "rear":
            values["enabled"] = self.rear_enabled.get()
        if self.travel_enabled:
            for key in RACK_LIMIT_FIELDS:
                values[key] = self.read_entry(key)
        values[TRAVEL] = 0.  # Validate the mechanism and stops independently of the current command.
        return validate_steering_settings(settings)

    def request(self, *, force_mapping=False, pending_export=None):
        if self.timer is not None:
            self.after_cancel(self.timer)
            self.timer = None
        axle = self.axle.get()
        self.update_slider()
        self.configuration = None
        try:
            settings = self.read_settings()
            self.configuration = settings[axle]
            travel = self.read_entry(TRAVEL) if self.travel_enabled else 0.
            pose = solve_steering(self.app.engine, axle, self.configuration, travel_mm=travel)
            metrics = {**pose["metrics"], **steering_camber_metrics(self.app.engine, axle, pose)}
        except (ValueError, ArithmeticError) as exc:
            self.state = self.pose = None
            self.notice.set(str(exc))
            self.tie_rod_z.set("—")
            self.update_results({})
            self.render(force_mapping=force_mapping)
            return None

        self.pose = pose
        settings[axle][TRAVEL] = metrics[TRAVEL]
        self.tie_rod_z.set(f"{metrics['tie_rod_z']:.6f}")
        self.state = {"metrics": {"axle": axle, **settings[axle], **metrics}}
        self.update_results(metrics)
        saved = steering_settings(self.app.project)
        if settings != saved:
            self.notice.set("正在預覽草稿。套用後可隨專案儲存。")
        elif "steering" in self.app.project:
            self.notice.set("已套用至目前專案。")
        else:
            self.notice.set("使用依目前車輛尺寸產生的初始值。")
        if pending_export is not None:
            self.pending_export = (self.configuration_key(), pending_export)
        self.render(force_mapping=force_mapping)
        return settings

    def update_results(self, metrics):
        def number(key, precision=3):
            value = metrics.get(key)
            return "—" if value is None else f"{value:.{precision}f}"

        for key, label, per_wheel in RESULT_ROWS:
            left = right = value = ""
            if per_wheel:
                precision = 5 if "gain" in key else 3
                left, right = (number(f"{side}_{key}", precision) for side in ("left", "right"))
            elif key == "turn":
                turn = {"left": "左轉", "right": "右轉"}.get(metrics.get("turn_direction"))
                value = (f"{turn} · {metrics['inner_corner']} / {metrics['outer_corner']}" if turn else "—")
            elif key == "ackermann_pct":
                value = number(key, 2)
            else:
                value = number(key)
            self.metrics.item(key, values=(label, left, right, value))

    def configuration_key(self):
        if self.configuration is None:
            return None
        fields = (*GEOMETRY_FIELDS, *RACK_LIMIT_FIELDS)
        return (self.app.revision, self.axle.get(), self.travel_enabled,
                *(self.configuration[field] for field in fields))

    def render(self, *, force_mapping=False):
        plot = self.plot_key
        geometry = plot == TV_PLOTS[0][0]
        lap_available = not geometry and self.app.lap_data is not None
        self.lap_check.state(["!disabled" if lap_available else "disabled"])
        lap = self.app.lap_data if lap_available and self.app.show_lap.get() else None
        key = self.configuration_key()
        job = (key, lap)
        self.mapping = self.sweep_cache[1] if self.sweep_cache and self.sweep_cache[0] == key else None
        overlay = self.overlay_cache[1] if lap is not None and self.overlay_cache and self.overlay_cache[0] == job else None
        if self.pending_export and self.pending_export[0] != key:
            self.pending_export = None
        if self.pending_export and self.mapping is not None:
            _, (kind, path) = self.pending_export
            self.pending_export = None
            self.app.perform_export(kind, path, self)
        needs_data = key is not None and (self.mapping is None or (lap is not None and overlay is None))
        message = ""
        if needs_data and (not geometry or force_mapping or self.pending_export is not None):
            if self.analysis_error and self.analysis_error[0] == job:
                message = self.analysis_error[1]
            else:
                if self.queued != job:
                    self.queue_analysis(job, self.mapping)
                message = "計算齒條行程掃描與 LapSim 工況…" if lap is not None else "計算齒條行程掃描…"
        if overlay is not None:
            description = "LapSim 點的 Rack 位移由左右輪轉角估算。"
            if plot in ("camber", "camber_gain", "steering_camber_gain"):
                description += "Camber 與增益點採此位移的靜態模型值。"
            message = overlay.get("reason") or description
        self.plot_notice.set(message)

        if geometry:
            spec = steering_chart(self.pose) if self.pose else None
        elif self.mapping is not None:
            spec = steering_sweep_chart(self.axle.get(), plot, self.mapping,
                                        metrics=self.state["metrics"] if self.state else None, lap=overlay)
        else:
            spec = None
        chart_key = (plot, key, id(overlay) if overlay is not None else None)
        self.chart.set(spec, preserve_view=self.chart_key == chart_key)
        self.chart_key = chart_key
        self.ready_for_export = self.state is not None or (not geometry and self.mapping is not None)

    def queue_analysis(self, job, mapping):
        key, lap = job
        axle, engine, values = self.axle.get(), self.app.engine, self.configuration.copy()
        self.queued = job
        self.token += 1
        token, revision = self.token, self.app.revision

        def work():
            if token != self.token or self.app.closing:
                return
            sweep = overlay = error = None
            try:
                sweep = mapping if mapping is not None else calculate_tv_sweep(engine, axle, values)
                if lap is not None and token == self.token:
                    overlay = calculate_tv_lap_overlay(engine, axle, lap, sweep)
            except Exception as exc:
                error = str(exc)
            if token == self.token and not self.app.closing:
                self.app.results.put((self, token, revision, job, (sweep, overlay), error))

        self.app.jobs.put(work)

    def receive(self, job, result, error):
        self.queued = None
        key, lap = job
        if key != self.configuration_key():
            return
        sweep, overlay = result
        if sweep is not None:
            self.sweep_cache = (key, sweep)
        if lap is not None and overlay is not None:
            self.overlay_cache = (job, overlay)
        self.analysis_error = (job, error) if error else None
        if error and self.pending_export and sweep is None:
            self.pending_export = None
            self.app.status.set(error)
        self.render()

    def apply(self):
        settings = self.request()
        if settings is None:
            return
        try:
            self.app.apply_steering(settings)
        except (ValueError, ArithmeticError) as exc:
            self.notice.set(str(exc))
            return
        self.notice.set("已套用至目前專案。")

    def destroy(self):
        if self.timer is not None:
            self.after_cancel(self.timer)
            self.timer = None
        super().destroy()
