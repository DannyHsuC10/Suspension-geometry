"""Standalone native workbench; workers never access Tk widgets."""
import copy
import csv
from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import ttk, filedialog, font, messagebox, simpledialog

from .widgets import ScrollFrame
from .project import (FIELDS, DERIVED_AXLE_FIELDS, MOUNT_X_FIELDS, mount_x_settings, travel_limits,
                      default_project, save_project, validate)
from .project_import import fill_import_project, read_import_project
from .engine import Engine, Unreachable
from .analysis import (RESULT_LABELS, LAP_CAMBER_PLOT, LAP_CAMBER_SERIES_BY_PAGE, PAGES, PLOTS, NOTES,
                       calculate_mapping, make_plot, static_point_rows)
from .charts import LabChart
from .model_notes import MODEL_NOTES
from .lap_data import LAP_PAGES, calculate_lap_camber, calculate_lap_metrics, read_lap_data
from .tire_page import TirePanel
from .export import STEP_PAGES, point_rows, write_report, write_csv, write_step
from . import hardpoints
from .input_limits import FIELD_LIMITS, INPUT_STEPS, NAME_MAX_LENGTH, range_hint, round_design_input
from .roll_center_search import SearchSettings
from .roll_center_dialog import RollCenterSearchDialog
from .fvsa_search import prepare_lap, validate_camber_limit
from .fvsa_dialog import FVSASearchDialog
from .step_dialog import StepExportDialog
from .svsa_search import SVSASearchSettings
from .svsa_dialog import SVSASettingsDialog, SVSASearchDialog
from .caster_search import validate_caster_lap
from .caster_dialog import CasterSearchDialog

ROOT = Path(__file__).resolve().parent.parent
PROJECTS = ROOT / "projects"
FRAME_INTERVAL_MS = 16
NON_KINEMATIC_VEHICLE_FIELDS = ("brake_bias_front", "drive_force_front", "roll_stiffness_ratio")
SV_ANTI_HINT = ("Anti 輸入與 SVSA 以朝另一軸為水平正向。前軸兩欄都填正值時，瞬心在接地點後上方。"
                "結果表沿用同一正向，並依煞車或驅動力分配加權。")


def show_input_range(parent, entry, var, group, key):
    """Allow intermediate typing; flag drafts and reject them on commit."""
    hint = range_hint(group, key, FIELDS[group][key][1])
    label = ttk.Label(parent, text=hint, foreground="#64748b", wraplength=280)
    label.pack(fill="x", pady=(0, 4))
    label.bind("<Configure>", lambda event: label.configure(wraplength=max(180, event.width-4)))
    entry.range_label = label

    def refresh(*_):
        readonly = str(entry.cget("state")) in ("disabled", "readonly")
        try:
            valid = readonly or FIELD_LIMITS[group][key].accepts(float(var.get()))
        except ValueError:
            valid = False
        entry.configure(style="TEntry" if valid else "Invalid.TEntry")
        label.configure(text=hint + ("；請修正輸入" if not valid else ""),
                        foreground="#64748b" if valid else "#b45309")

    var.trace_add("write", refresh)
    entry.bind("<FocusOut>", refresh, add=True)
    refresh()


def check_mount_inputs(engine):
    """Preview may fall back to safe extrema; committing must be explicit."""
    if hardpoints.enabled(engine.project):
        return
    for axle, checks in engine.scene.geometry.get("mount_position_checks", {}).items():
        for key, check in checks.items():
            if check["requested_x"] is not None and check["fallback"]:
                raise ValueError(f"{axle} · {MOUNT_X_FIELDS[key]}：{check['message']} 請修正輸入，或留白使用極限。")


def pose_settings(project):
    """Compare geometry and sweep inputs while ignoring force/balance settings."""
    settings = dict(project)
    settings["vehicle"] = {key: value for key, value in project["vehicle"].items()
                           if key not in NON_KINEMATIC_VEHICLE_FIELDS}
    if not hardpoints.enabled(project):
        settings.pop("hardpoints", None)
    settings["arm_mount_x"] = mount_x_settings(project)
    return settings


def field_entries(parent, project, groups, widgets=None, actions=None):
    variables = {}
    for title, group, keys in groups:
        ttk.Label(parent, text=title, font=("Microsoft JhengHei UI", 11, "bold"), padding=(0, 12, 0, 6)).pack(anchor="w")
        for key in keys:
            label, unit = FIELDS[group][key]
            row = ttk.Frame(parent)
            row.pack(fill="x", pady=2)
            action = (actions or {}).get((group, key))
            ttk.Label(row, text=f"{label} ({unit})", wraplength=215).pack(
                **({"anchor": "w"} if action else {"side": "left", "fill": "x", "expand": True}))
            if action:
                controls = ttk.Frame(row)
                controls.pack(fill="x", pady=(2, 0))
            else:
                controls = row
            var = tk.StringVar(value=f"{project[group][key]:.10g}")
            entry = ttk.Entry(controls, textvariable=var, width=12)
            if action:
                entry.pack(side="left", fill="x", expand=True)
                text, command = action
                entry.action_button = ttk.Button(controls, text=text, command=command, style="RCSearch.TButton")
                entry.action_button.pack(side="right", padx=(5, 0))
            else:
                entry.pack(side="right", padx=(6, 0))
            show_input_range(parent, entry, var, group, key)
            if widgets is not None:
                widgets[(group, key)] = entry
            variables[(group, key)] = var
    return variables


def read_entries(project, entries):
    result = copy.deepcopy(project)
    direct = hardpoints.enabled(project)
    for (group, key), var in entries.items():
        if direct and group in ("front", "rear") and key != "static_camber_deg":
            continue
        # Entries show a readable precision. Preserve the full stored value
        # when that display was not edited, so changing drive share alone
        # cannot round SVSA or the other geometric inputs.
        if var.get().strip() == f"{project[group][key]:.10g}":
            continue
        try:
            result[group][key] = round_design_input(group, key, float(var.get()))
        except ValueError as exc:
            raise ValueError(f"{group} · {FIELDS[group][key][0]} 需要數值。") from exc
    samples = result["analysis"]["samples"]
    result["analysis"]["samples"] = int(samples) if float(samples).is_integer() else samples
    return validate(result)


class ImportRepairDialog(tk.Toplevel):
    def __init__(self, app, project, fields):
        super().__init__(app)
        self.app, self.project, self.fields = app, project, fields
        self.applied = False
        self.title("補齊專案缺失欄位")
        self.geometry(f"900x{min(600, 225 + 46 * len(fields))}")
        self.minsize(780, 260)
        self.transient(app)
        self.bind("<Escape>", lambda event: self.destroy())
        header = ttk.Frame(self, padding=(14, 8, 14, 4))
        header.pack(fill="x")
        ttk.Button(header, text="取消", command=self.destroy).pack(side="right")
        self.apply_button = ttk.Button(header, text="補齊並匯入", command=self.apply)
        self.apply_button.pack(side="right", padx=8)
        ttk.Label(header, text=f"找到 {len(fields)} 個缺失欄位",
                  font=("Microsoft JhengHei UI", 13, "bold")).pack(side="left")
        ttk.Label(self, text="逐格輸入數值或文字，留空就採用該欄的預設值。已有資料會保留。",
                  padding=(14, 0, 14, 10)).pack(anchor="w")
        self.error = ttk.Label(self, foreground="#9a3412", wraplength=745, padding=(14, 4))
        self.error.pack(fill="x")

        scroll = ScrollFrame(self, padding=(14, 0))
        scroll.pack(fill="both", expand=True)
        self.canvas = scroll.canvas
        self.canvas.unbind("<MouseWheel>")
        self.bind("<MouseWheel>", self.scroll_wheel)
        table = scroll.content
        table.columnconfigure(0, weight=1)
        for col, heading in enumerate(("缺失欄位／JSON 路徑", "預設值", "自行輸入（可留空）")):
            ttk.Label(table, text=heading, padding=(6, 4)).grid(row=0, column=col, sticky="w")
        self.inputs = []
        for row, field in enumerate(fields, start=1):
            title = f"{field.label} ({field.unit})" if field.unit else field.label
            ttk.Label(table, text=f"{title}\n{field.location}", padding=(6, 4),
                      foreground="#334155").grid(row=row, column=0, sticky="w")
            ttk.Label(table, text=field.default_text(), wraplength=220,
                      padding=(6, 4)).grid(row=row, column=1, sticky="w")
            if isinstance(field.default, bool):
                entry = ttk.Combobox(table, width=20, state="readonly", values=("", "是", "否"))
            else:
                entry = ttk.Entry(table, width=23)
            entry.grid(row=row, column=2, padx=6, sticky="ew")
            entry.bind("<FocusIn>", self.show_focused_input)
            self.inputs.append(entry)

        self.wait_visibility()
        self.grab_set()
        self.inputs[0].focus_set()

    def scroll_wheel(self, event):
        self.canvas.yview_scroll(-int(event.delta / 120), "units")

    def show_focused_input(self, event):
        bounds = self.canvas.bbox("all")
        if not bounds:
            return
        top = event.widget.winfo_y()
        bottom = top + event.widget.winfo_height()
        visible_top = self.canvas.canvasy(0)
        height = self.canvas.winfo_height()
        if top < visible_top:
            self.canvas.yview_moveto(top / bounds[3])
        elif bottom > visible_top + height:
            self.canvas.yview_moveto((bottom - height) / bounds[3])

    def apply(self):
        try:
            candidate = fill_import_project(self.project, self.fields, [entry.get() for entry in self.inputs])
            self.app.apply_project(candidate, preserve_pose=False)
        except (ValueError, ArithmeticError) as exc:
            self.error.configure(text=str(exc))
            return
        self.applied = True
        self.destroy()


class ParameterEditor(tk.Toplevel):
    def __init__(self, app):
        super().__init__(app)
        self.app = app
        self._preview_timer = None
        self._preview_traces = []
        self._mount_positions_changed = False
        self.title("共用車輛／懸吊設定 · SuspensionGeometryLab")
        self.geometry("1060x820")
        self.minsize(940, 700)
        self.transient(app)
        self.grab_set()
        ttk.Label(self, text="所有頁面共用一份 JSON 設定；一般長度 m、角度 deg、比例 1=100%。硬點與 A 臂車身 X 欄位使用 mm。", padding=14).pack(fill="x")
        name_row = ttk.Frame(self, padding=(14, 0, 14, 10))
        name_row.pack(fill="x")
        ttk.Label(name_row, text="專案名稱").pack(side="left")
        self.name = tk.StringVar(value=app.project["name"])
        ttk.Entry(name_row, textvariable=self.name).pack(side="left", fill="x", expand=True, padx=10)
        ttk.Label(name_row, text=f"1～{NAME_MAX_LENGTH} 字元").pack(side="right")
        self.hardpoint_mode = tk.BooleanVar(value=hardpoints.enabled(app.project))
        self.point_data = copy.deepcopy(app.project["hardpoints"]["corners"]) if "hardpoints" in app.project else hardpoints.from_scene(app.engine.scene)
        self.points_edited = "hardpoints" in app.project
        self.hardpoint_check = ttk.Checkbutton(
            self, text="使用直接修改硬點 XYZ（取消勾選：使用懸吊參數解算）", variable=self.hardpoint_mode,
            command=self.change_geometry_mode)
        self.hardpoint_check.pack(anchor="w", padx=14, pady=(0, 10))
        notebook = ttk.Notebook(self)
        self.notebook = notebook
        notebook.pack(fill="both", expand=True, padx=14)
        self.entries, self.entry_widgets = {}, {}
        self.mount_x_entries, self.mount_x_widgets, self.mount_x_labels = {}, {}, {}
        for group, label in (("vehicle", "車輛 / 輪胎"), ("front", "前軸參數模式"), ("rear", "後軸參數模式"), ("analysis", "掃描範圍")):
            frame = ScrollFrame(notebook)
            notebook.add(frame, text=label)
            self.entries.update(field_entries(frame.content, app.project, [(label, group, FIELDS[group])],
                                              self.entry_widgets))
            if group == "vehicle":
                ttk.Label(frame.content, text="加速比例為前軸驅動力占比：0 後驅、1 前驅，後軸自動為 1−此值；與煞車比例獨立。",
                          wraplength=850, foreground="#475569").pack(fill="x", pady=8)
                ttk.Label(frame.content, text="前後總側傾剛性比包含彈簧與防傾桿，使用換算到輪端的軸側傾剛性（同單位）；"
                          "1 代表前後相同。後軸 RC 依解析式求解前後載荷轉移量相等（各占 50%）。",
                          wraplength=850, foreground="#475569").pack(fill="x", pady=8)
            if group in ("front", "rear"):
                ttk.Label(frame.content, text=SV_ANTI_HINT,
                          wraplength=850, foreground="#475569").pack(fill="x", pady=8)
                self.build_mount_x_fields(frame.content, group)
        self.hardpoint_tab = ttk.Frame(notebook, padding=10)
        notebook.insert(1, self.hardpoint_tab, text="24 個懸吊硬點")
        self.build_hardpoint_tab()
        ttk.Label(self, text="硬點模式保留原始 XYZ，參數由幾何反算；參數模式保留原設計輸入。兩模式皆由此視窗套用至全部頁面。", wraplength=980, padding=14).pack(fill="x")
        buttons = ttk.Frame(self, padding=14)
        buttons.pack(fill="x")
        ttk.Button(buttons, text="取消", command=self.destroy).pack(side="right")
        ttk.Button(buttons, text="套用全部設定", command=self.apply).pack(side="right", padx=8)
        self.change_geometry_mode()
        if not self.hardpoint_mode.get():
            self.notebook.select(self.hardpoint_tab)
        for var in [self.name, *self.entries.values()]:
            trace = var.trace_add("write", self.schedule_derived)
            self._preview_traces.append((var, trace))
        for var in self.mount_x_entries.values():
            trace = var.trace_add("write", self.schedule_mount_positions)
            self._preview_traces.append((var, trace))

    def build_mount_x_fields(self, parent, axle):
        ttk.Label(parent, text="A 臂車身接點 · 全車縱向 X", font=("Microsoft JhengHei UI", 11, "bold"),
                  padding=(0, 16, 0, 6)).pack(anchor="w")
        ttk.Label(parent, text="單位 mm；空白使用輪圈避讓極限。X 向前，後接點 X 必須小於前接點 X。\n"
                  "沿用 FV 接點指定的全車 Y，以 A 臂平面解 Z；違規時預覽退回極限，套用前須修正或留白。",
                  wraplength=850).pack(anchor="w", pady=(0, 6))
        for key, label in MOUNT_X_FIELDS.items():
            field = (axle, key)
            row = ttk.Frame(parent)
            row.pack(fill="x", pady=(6, 0))
            ttk.Label(row, text=label+" (mm)").pack(side="left", fill="x", expand=True)
            value = mount_x_settings(self.app.project)[axle][key]
            var = tk.StringVar(value="" if value is None else f"{value*1000:.12g}")
            entry = ttk.Entry(row, textvariable=var, width=16)
            entry.pack(side="right", padx=(6, 0))
            status = ttk.Label(parent, text="空白：自動採用幾何極限接點。", wraplength=850, foreground="#475569")
            status.pack(fill="x", pady=(2, 4))
            self.mount_x_entries[field], self.mount_x_widgets[field], self.mount_x_labels[field] = var, entry, status

    def build_hardpoint_tab(self):
        frame = self.hardpoint_tab
        ttk.Label(frame, text="X 向前、Y 向左、Z 向上；前軸 X=0、後軸 X=−軸距。表格單位 mm。\n"
                  "每輪上／下 A 臂各三點。雙擊任一硬點編輯 XYZ，對側同名點立即鏡射（X、−Y、Z）。",
                  padding=(0, 0, 0, 10)).pack(anchor="w")
        body = ttk.Frame(frame)
        body.pack(fill="both", expand=True)
        left = ttk.Frame(body)
        left.pack(side="left", fill="both", expand=True)
        self.hardpoint_tree = ttk.Treeview(left, columns=("x", "y", "z"), show="tree headings", selectmode="browse")
        self.hardpoint_tree.heading("#0", text="硬點／雙擊編輯")
        self.hardpoint_tree.column("#0", width=230, minwidth=180)
        for axis in ("x", "y", "z"):
            self.hardpoint_tree.heading(axis, text=axis.upper()+" (mm)")
            self.hardpoint_tree.column(axis, width=90, minwidth=75, anchor="e")
        scrollbar = ttk.Scrollbar(left, command=self.hardpoint_tree.yview)
        self.hardpoint_tree.configure(yscrollcommand=scrollbar.set)
        self.hardpoint_tree.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        self.hardpoint_tree.bind("<Double-1>", self.edit_hardpoint)
        right = ttk.Frame(body, padding=(12, 0, 0, 0))
        right.pack(side="left", fill="y")
        ttk.Label(right, text="硬點反算靜態結果（唯讀）").pack(anchor="w", pady=(0, 6))
        self.derived_tree = ttk.Treeview(right, columns=("front", "rear"), show="tree headings", height=15)
        self.derived_tree.heading("#0", text="參數／單位")
        self.derived_tree.column("#0", width=180)
        for axle, label in (("front", "前軸"), ("rear", "後軸")):
            self.derived_tree.heading(axle, text=label)
            self.derived_tree.column(axle, width=85, anchor="e")
        self.derived_tree.pack(fill="y", expand=True)
        controls = ttk.Frame(frame, padding=(0, 10, 0, 0))
        controls.pack(fill="x")
        ttk.Button(controls, text="由目前參數重新產生 24 點", command=self.regenerate_hardpoints).pack(side="left")
        self.derivation_message = tk.StringVar()
        ttk.Label(frame, textvariable=self.derivation_message, wraplength=980, foreground="#475569",
                  padding=(0, 10, 0, 0)).pack(fill="x")
        ttk.Label(frame, text="輪心與輪胎位置沿用車輛／輪距／負載半徑設定。FV 使用等效樞點；SV 使用硬點軸線的切線近似；"
                  "WE3D 繞 UBJ–LBJ 主銷轉向。六點未包含轉向拉桿，不求解 bump steer 或行程與轉向的完整 3D 耦合。",
                  wraplength=980, foreground="#475569", padding=(0, 8, 0, 0)).pack(fill="x")
        self.populate_hardpoints()

    def candidate(self, mode=None):
        base = copy.deepcopy(self.app.project)
        base["arm_mount_x"] = {axle: {} for axle in ("front", "rear")}
        for (axle, key), var in self.mount_x_entries.items():
            raw = var.get().strip()
            stored = mount_x_settings(self.app.project)[axle][key]
            displayed = "" if stored is None else f"{stored*1000:.12g}"
            try:
                value = stored if raw == displayed else float(raw)/1000 if raw else None
            except ValueError as exc:
                self.mount_x_labels[(axle, key)].configure(text="請輸入有效的 X 數值（mm），或留白使用極限。", foreground="#b45309")
                raise ValueError(f"{axle} · {MOUNT_X_FIELDS[key]} 需要數值。") from exc
            base["arm_mount_x"][axle][key] = value
        # These entries contain the retained design inputs, even in hardpoint mode.
        # Panel entries, in contrast, show derived values and are skipped there.
        base["hardpoints"] = {"enabled": False, "corners": copy.deepcopy(self.point_data)}
        result = read_entries(base, self.entries)
        result["hardpoints"]["enabled"] = self.hardpoint_mode.get() if mode is None else mode
        result["name"] = self.name.get().strip()
        return validate(result)

    def change_geometry_mode(self):
        if self.hardpoint_mode.get() and not self.points_edited:
            if not self.regenerate_hardpoints():
                self.hardpoint_mode.set(False)
        for (group, key), entry in self.entry_widgets.items():
            if group in ("front", "rear") and key != "static_camber_deg":
                entry.configure(state="disabled" if self.hardpoint_mode.get() else "normal")
        for entry in self.mount_x_widgets.values():
            entry.configure(state="disabled" if self.hardpoint_mode.get() else "normal")
        self.hardpoint_tree.state(["!disabled"] if self.hardpoint_mode.get() else ["disabled"])
        if self.hardpoint_mode.get():
            self.notebook.select(self.hardpoint_tab)
        self.refresh_derived()

    def populate_hardpoints(self):
        tree = self.hardpoint_tree
        selection = tree.selection()
        tree.delete(*tree.get_children())
        for corner in hardpoints.CORNERS:
            tree.insert("", "end", iid=corner, text=corner, open=True)
            for key, label in hardpoints.POINTS.items():
                tree.insert(corner, "end", iid=corner+"/"+key, text=key+" · "+label,
                            values=[f"{value*1000:.4f}" for value in self.point_data[corner][key]])
        if selection and tree.exists(selection[0]):
            tree.selection_set(selection)

    def set_hardpoint(self, corner, name, coordinates):
        hardpoints.set_point(self.point_data, corner, name, coordinates)
        self.points_edited = True
        self.populate_hardpoints()
        self.refresh_derived()

    def edit_hardpoint(self, event):
        row = self.hardpoint_tree.identify_row(event.y)
        if not self.hardpoint_mode.get() or "/" not in row:
            return
        corner, name = row.split("/")
        point = self.point_data[corner][name]
        raw = simpledialog.askstring("編輯懸吊硬點", f"{corner}/{name}：X, Y, Z（mm）；對側會同步鏡射。\n不設數值上下界；請輸入三個有限座標。",
                                     initialvalue=", ".join(f"{x*1000:.12g}" for x in point), parent=self)
        if raw is None:
            return
        try:
            coordinates = [float(x.strip())/1000 for x in raw.replace("，", ",").split(",")]
            self.set_hardpoint(corner, name, coordinates)
        except ValueError as exc:
            messagebox.showerror("硬點座標錯誤", str(exc), parent=self)

    def regenerate_hardpoints(self):
        try:
            engine = Engine(self.candidate(False))
            self.point_data = hardpoints.from_scene(engine.scene)
            self.points_edited = False
            self.populate_hardpoints()
            self.refresh_derived()
            return True
        except (ValueError, ArithmeticError) as exc:
            messagebox.showerror("無法由參數產生硬點", str(exc), parent=self)
            return False

    def refresh_derived(self):
        if self._preview_timer is not None:
            self.after_cancel(self._preview_timer)
            self._preview_timer = None
        self.derived_tree.delete(*self.derived_tree.get_children())
        try:
            parameter_engine = Engine(self.candidate(False))
            checks = parameter_engine.scene.geometry["mount_position_checks"]
            for (axle, key), label in self.mount_x_labels.items():
                check = checks[axle][key]
                if check["limit_min_x"] is None:
                    text = check["message"]
                else:
                    text = (f"極限 X：{check['limit_min_x']*1000:.6f} ～ {check['limit_max_x']*1000:.6f} mm；"
                            f"實際使用 X：{check['actual_x']*1000:.6f} mm。")
                    if check["message"]:
                        text += "\n"+check["message"]
                label.configure(text=text, foreground="#b45309" if check["fallback"] else "#475569")
            if self._mount_positions_changed and not self.points_edited and not self.hardpoint_mode.get():
                self.point_data = hardpoints.from_scene(parameter_engine.scene)
                self.populate_hardpoints()
            self._mount_positions_changed = False
            engine = Engine(self.candidate(True))
            for key, (label, unit) in DERIVED_AXLE_FIELDS.items():
                values = [engine.derived_parameters[axle][key] for axle in ("front", "rear")]
                self.derived_tree.insert("", "end", iid=key, text=f"{label} ({unit})",
                                         values=["N/A" if value is None else f"{value:.7g}" for value in values])
            self.derivation_message.set("硬點幾何有效。N/A 表示平行投影沒有有限瞬心／中心。" +
                                        ("目前使用硬點模式。" if self.hardpoint_mode.get() else "目前使用參數模式；硬點表為草稿，勾選後才使用。"))
        except (ValueError, ArithmeticError) as exc:
            self.derivation_message.set("硬點草稿尚無法套用："+str(exc))

    def schedule_derived(self, *_):
        if self._preview_timer is not None:
            self.after_cancel(self._preview_timer)
        self._preview_timer = self.after(200, self.refresh_derived)

    def schedule_mount_positions(self, *_):
        self._mount_positions_changed = True
        self.schedule_derived()

    def destroy(self):
        if self._preview_timer is not None:
            self.after_cancel(self._preview_timer)
            self._preview_timer = None
        for var, trace in self._preview_traces:
            var.trace_remove("write", trace)
        self._preview_traces.clear()
        super().destroy()

    def apply(self):
        try:
            candidate = self.candidate()
            self.app.apply_project(candidate)
            self.destroy()
        except (ValueError, ArithmeticError) as exc:
            messagebox.showerror("設定無法套用", str(exc), parent=self)


class ModelNotes(ttk.Frame):
    """Collapsible, selectable explanation with its own scrolling area."""

    def __init__(self, master, chart, sections):
        super().__init__(master)
        self.chart = chart
        self.resize_job = None
        self.toggle = ttk.Button(self, text="▸ 展開模型說明", command=self.toggle_content)
        self.toggle.pack(anchor="w")
        self.body = ttk.Frame(self)
        self.text = tk.Text(
            self.body, font=("Microsoft JhengHei UI", 10), wrap="word", width=1, height=6,
            background="#ffffff", foreground="#0f172a", padx=12, pady=8,
            relief="flat", highlightthickness=1, highlightbackground="#cbd5e1",
            highlightcolor="#0f766e", takefocus=True,
        )
        scrollbar = ttk.Scrollbar(self.body, orient="vertical", command=self.text.yview)
        self.text.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.text.pack(side="left", fill="both", expand=True)
        self.text.tag_configure("section", font=("Microsoft JhengHei UI", 10, "bold"), spacing1=7, spacing3=4)
        self.text.tag_configure("paragraph", spacing3=6)
        self.text.tag_configure("equation", foreground="#1e3a5f", lmargin1=16, lmargin2=16,
                                spacing3=6, wrap="char")
        for heading, paragraphs in sections:
            self.text.insert("end", heading + "\n", "section")
            for paragraph in paragraphs:
                is_equation = paragraph.startswith("  ")
                if is_equation:
                    paragraph = "\n".join(line.removeprefix("  ") for line in paragraph.splitlines())
                self.text.insert("end", paragraph + "\n", "equation" if is_equation else "paragraph")
        self.text.configure(state="disabled")
        self.text.bind("<Shift-Tab>", self.focus_toggle)
        self.line_height = font.Font(root=self, font=self.text.cget("font")).metrics("linespace")
        self.chart.bind("<Configure>", self.schedule_resize, add=True)
        self.chart.bind("<Unmap>", self.schedule_resize, add=True)
        self.master.bind("<Configure>", self.schedule_resize, add=True)

    def focus_toggle(self, _event):
        self.toggle.focus_set()
        return "break"

    def schedule_resize(self, _event):
        if self.body.winfo_ismapped() and self.resize_job is None:
            self.resize_job = self.after_idle(self.resize)

    def resize(self):
        self.resize_job = None
        # Reclaim the space currently occupied by the reader when budgeting
        # how much of the chart area the expanded notes should use.
        height = self.chart.winfo_height() if self.chart.winfo_ismapped() else 0
        if self.body.winfo_ismapped():
            height += self.body.winfo_height() + 4
        budget = min(height // 2, int(self.master.winfo_height() * .26))
        lines = max(2, min(14, (budget - 22) // self.line_height))
        if lines != int(self.text.cget("height")):
            self.text.configure(height=lines)

    def toggle_content(self):
        if self.body.winfo_manager():
            if self.resize_job is not None:
                self.after_cancel(self.resize_job)
                self.resize_job = None
            if self.focus_get() == self.text:
                self.toggle.focus_set()
            self.body.pack_forget()
            self.toggle.configure(text="▸ 展開模型說明")
        else:
            self.resize()
            self.body.pack(fill="x", pady=(4, 0))
            self.toggle.configure(text="▾ 收合模型說明")

    def destroy(self):
        if self.resize_job is not None:
            self.after_cancel(self.resize_job)
        super().destroy()


class Panel(ttk.Frame):
    def __init__(self, app, key):
        super().__init__(app.notebook)
        self.app, self.key = app, key
        self.token = 0
        self.timer = None
        self._request_in_flight = False
        self._slider_pending = False
        self.state = self.mapping = self.error = None
        self._chart_key = None
        self.ready = False
        self.pending_export = None
        self.axle = tk.StringVar(value="front")
        self.mode = tk.StringVar(value="左右行程" if key in ("front", "rear") else "前後行程")
        self.plot = tk.StringVar(value=PLOTS[key][0][1])
        self.controls = [tk.DoubleVar(value=0.), tk.DoubleVar(value=0.)]
        self.groups = {k: tk.BooleanVar(value=True) for k in ("front", "rear", "sv")}
        self.construction = tk.BooleanVar(value=True)
        self.static_references = tk.BooleanVar(value=True)
        pane = ttk.Panedwindow(self, orient="horizontal")
        pane.pack(fill="both", expand=True)
        left = ttk.Frame(pane, width=345)
        pane.add(left, weight=0)
        right = ttk.Frame(pane)
        pane.add(right, weight=1)
        self.scroll = ScrollFrame(left)
        self.scroll.pack(fill="both", expand=True, padx=(10, 4))
        self.entry_widgets = {}
        self.rc_search_dialog = None
        self.fvsa_search_dialog = None
        self.svsa_search_dialog = None
        self.svsa_search_settings = SVSASearchSettings()
        self.caster_search_dialog = None
        if key == "rear":
            self.balance_context = ttk.Label(self.scroll.content, text="", wraplength=295, foreground="#475569")
            self.balance_context.pack(fill="x", pady=(10, 0))
        actions = ({(key, "rc_height"): ("尋找最佳roll center", self.find_roll_center),
                    (key, "fvsa"): ("尋找最佳FVSA", self.find_fvsa)}
                   if key in ("front", "rear") else
                   {(axle, "svsa"): ("尋找最佳SVSA", self.find_svsa) for axle in ("front", "rear")}
                   if key == "sv" else None)
        if key in ("sv", "we"):
            actions = {**(actions or {}), ("front", "caster_deg"): ("尋找最佳Caster", self.find_front_caster)}
        self.entries = field_entries(self.scroll.content, app.project, self.field_groups(), self.entry_widgets, actions)
        if key == "rear":
            for field in ("wheelbase", "cg_to_front", "roll_stiffness_ratio"):
                self.entries[("vehicle", field)].trace_add("write", self.update_balance_context)
            self.update_balance_context()
        self.geometry_hint = ttk.Label(self.scroll.content, text="", wraplength=295, foreground="#475569")
        self.geometry_hint.pack(fill="x", pady=(8, 0))
        if key in ("sv", "static"):
            ttk.Label(self.scroll.content, text=SV_ANTI_HINT, wraplength=295,
                      foreground="#64748b", padding=(0, 12)).pack(fill="x")
        ttk.Button(left, text="套用左側參數並重新解算", command=self.apply).pack(fill="x", padx=10, pady=10)
        toolbar = ttk.Frame(right, padding=(8, 10))
        toolbar.pack(fill="x")
        ttk.Label(toolbar, text="圖表").pack(side="left")
        self.plot_select = ttk.Combobox(toolbar, state="readonly", textvariable=self.plot,
                                       values=[name for _, name in PLOTS[key]], width=40)
        self.plot_select.pack(side="left", padx=8)
        self.plot_select.bind("<<ComboboxSelected>>", lambda e: self.request())
        ttk.Button(toolbar, text="重設視角", command=lambda: self.chart.reset()).pack(side="left", padx=4)
        if key == "we":
            self.axle_controls = ttk.Frame(toolbar)
            self.axle_controls.pack(side="left")
            ttk.Label(self.axle_controls, text="車軸").pack(side="left", padx=(8, 0))
            combo = ttk.Combobox(self.axle_controls, state="readonly", textvariable=self.axle, values=["front", "rear"], width=8)
            combo.pack(side="left", padx=6)
            combo.bind("<<ComboboxSelected>>", lambda e: self.request())
        if key in ("front", "rear", "sv"):
            row = ttk.Frame(right, padding=(8, 0, 8, 0))
            row.pack(fill="x")
            modes = ["前後行程", "Heave / Pitch"] if key == "sv" else ["左右行程", "Heave / Roll"]
            ttk.Label(row, text="滑桿 / 曲面座標").pack(side="left")
            combo = ttk.Combobox(row, state="readonly", textvariable=self.mode, values=modes, width=18)
            combo.pack(side="left", padx=8)
            combo.bind("<<ComboboxSelected>>", self.change_mode)
            hint = "角度範圍由上下行程自動推導" if key == "sv" else "Roll 由目前 Heave 與左右輪行程上下限控制"
            ttk.Label(row, text=hint, foreground="#64748b").pack(side="left", padx=8)
        if key == "static":
            row = ttk.Frame(right, padding=(8, 0))
            row.pack(fill="x")
            for group, label in (("front", "前 FV 平面"), ("rear", "後 FV 平面"), ("sv", "SV 平面")):
                ttk.Checkbutton(row, text=label, variable=self.groups[group], command=self.request).pack(side="left", padx=5)
            self.static_references_check = ttk.Checkbutton(
                row, text="瞬心／Kingpin 連線與延長線", variable=self.static_references, command=self.render)
            self.static_references_check.pack(side="left", padx=8)
        else:
            row = ttk.Frame(right, padding=(8, 4))
            row.pack(fill="x")
            ttk.Checkbutton(row, text="顯示 IC / 力線參考", variable=self.construction, command=self.render).pack(side="left")
            self.zero_button = ttk.Button(row, text="回到零姿態", command=self.zero)
            self.zero_button.pack(side="left", padx=12)
        if key in (*LAP_PAGES, "we"):
            row = ttk.Frame(right, padding=(8, 2))
            row.pack(fill="x")
            ttk.Button(row, text="匯入 LapSim CSV", command=app.open_lap).pack(side="left")
            if key in LAP_PAGES:
                self.lap_check = ttk.Checkbutton(row, text="顯示 LapSim 行程分布",
                                                 variable=app.show_lap, command=app.toggle_lap)
                self.lap_check.pack(side="left", padx=8)
                self.lap_check.state(["disabled"])
            ttk.Label(row, textvariable=app.lap_source, foreground="#475569").pack(side="left", padx=8)
        self.sliders, self.slider_labels = [], []
        for i in range(0 if key == "static" else 1 if key == "we" else 2):
            row = ttk.Frame(right, padding=(8, 2))
            row.pack(fill="x")
            label = ttk.Label(row, width=23)
            label.pack(side="left")
            self.slider_labels.append(label)
            scale = ttk.Scale(row, variable=self.controls[i], command=lambda value: self.slider_changed())
            scale.pack(side="left", fill="x", expand=True, padx=8)
            self.sliders.append(scale)
        self.configure_sliders()
        self.chart = LabChart(right)
        self.camber_filter = None
        if key in LAP_CAMBER_SERIES_BY_PAGE:
            self.camber_filter = ttk.Frame(right, padding=(8, 2))
            self.camber_visible = {}
            series = LAP_CAMBER_SERIES_BY_PAGE[key]
            stages = len(series)//2
            ttk.Label(self.camber_filter, text="顯示類別").grid(row=0, column=0, rowspan=2, padx=(0, 8))
            for i, (metric, label, _, _) in enumerate(series):
                variable = self.camber_visible[metric] = tk.BooleanVar(value=True)
                ttk.Checkbutton(self.camber_filter, text=label, variable=variable, command=self.render).grid(
                    row=i//stages, column=1+i%stages, sticky="w", padx=(0, 12))
            normal_row = ttk.Frame(self.camber_filter)
            normal_row.grid(row=2, column=0, columnspan=stages+1, sticky="w", pady=(3, 0))
            self.show_normal_curves = tk.BooleanVar(value=True)
            self.normal_curves_check = ttk.Checkbutton(normal_row, text="顯示常態分布曲線",
                                                       variable=self.show_normal_curves, command=self.render)
            self.normal_curves_check.pack(side="left")
        metric_frame = ttk.Frame(right)
        self.metrics = ttk.Treeview(metric_frame, columns=("metric", "value"), show="headings", height=5)
        self.metrics.heading("metric", text="目前姿態結果 / 單位")
        self.metrics.heading("value", text="數值（N/A：目前姿態無有效定義）")
        self.metrics.column("metric", width=270)
        self.metrics.column("value", width=240)
        self.metrics.pack(side="left", fill="both", expand=True)
        scrollbar = ttk.Scrollbar(metric_frame, command=self.metrics.yview)
        self.metrics.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.model_notes = ModelNotes(right, self.chart, MODEL_NOTES[key])
        self.model_notes.pack(side="bottom", fill="x", padx=8, pady=(0, 5))
        self.note = ttk.Label(right, text=NOTES[key], wraplength=950, foreground="#475569", padding=(8, 7))
        self.note.pack(side="bottom", fill="x")
        right.bind("<Configure>", lambda event: self.note.configure(wraplength=max(300, event.width-24)), add=True)
        metric_frame.pack(side="bottom", fill="x", padx=8)
        self.chart.pack(fill="both", expand=True, padx=8, pady=6)
        self.ready = True
        self.sync_geometry_fields()

    def field_groups(self):
        k = self.key
        v = ["track_"+k, "free_radius", "tire_width"] if k in ("front", "rear") else list(FIELDS["vehicle"]) if k == "static" else ["wheelbase", "cg_height", "ground_clearance", "free_radius", "tire_stiffness", "brake_bias_front", "drive_force_front"]
        if k == "rear":
            v = ["track_front", "track_rear", "wheelbase", "cg_to_front", "cg_height",
                 "roll_stiffness_ratio", "free_radius", "tire_width"]
        groups = [("共用車輛 / 輪胎", "vehicle", v)]
        if k in ("front", "rear"):
            groups.append((PAGES[k]+" 設計", k, [*list(FIELDS[k])[:2], "static_camber_deg", *list(FIELDS[k])[2:8]]))
        elif k in ("sv", "we"):
            for axle, title in (("front", "前軸設計"), ("rear", "後軸設計")):
                keys = (["caster_deg", "mechanical_trail", "anti", "svsa", "upper_offset", "lower_offset"]
                        if k == "sv" else ["kpi_deg", "caster_deg", "scrub_radius", "mechanical_trail", "static_camber_deg"])
                groups.append((title, axle, keys))
        else:
            for axle in ("front", "rear"):
                groups.append(("前軸" if axle == "front" else "後軸", axle, FIELDS[axle]))
        groups.append(("範圍掃描設定", "analysis", FIELDS["analysis"]))
        return groups

    def sync(self, keep_pose=False):
        if self.timer:
            self.after_cancel(self.timer)
            self.timer = None
        self._request_in_flight = self._slider_pending = False
        self.ready_for_export = False
        self.sync_geometry_fields()
        self.state = self.mapping = self.error = None
        self.token += 1
        self.pending_export = None
        self._chart_key = None
        if not keep_pose:
            for var in self.controls:
                var.set(0.)
        self.configure_sliders()
        self.chart.set(None)
        self.metrics.delete(*self.metrics.get_children())

    def sync_geometry_fields(self):
        direct = hardpoints.enabled(self.app.project)
        engine = self.app.engine
        for (group, key), var in self.entries.items():
            derived = direct and group in ("front", "rear") and key != "static_camber_deg"
            value = engine.derived_parameters[group][key] if derived else self.app.project[group][key]
            var.set("N/A" if value is None else f"{value:.10g}")
            self.entry_widgets[(group, key)].configure(state="disabled" if derived else "normal")
            if hasattr(self.entry_widgets[(group, key)], "action_button"):
                self.entry_widgets[(group, key)].action_button.state(["disabled"] if derived else ["!disabled"])
        self.geometry_hint.configure(text="硬點模式：懸吊參數為反算唯讀值；靜態 Camber 可獨立編輯，24 個硬點由上方共用設定修改。" if direct else "參數模式：先以 Camber＝0 建立機構，再調整車輪的靜態 Camber 安裝角。")
        self.note.configure(text=NOTES[self.key] + (" 硬點模式：FV 使用等效樞點，SV 沿硬點軸線的固定切線近似；靜態 3D 與 WE 使用原始 XYZ。" if direct else ""))
        if self.key == "rear":
            self.update_balance_context()

    def update_balance_context(self, *_):
        try:
            length = float(self.entries[("vehicle", "wheelbase")].get())
            distance = float(self.entries[("vehicle", "cg_to_front")].get())
            ratio = float(self.entries[("vehicle", "roll_stiffness_ratio")].get())
            if not (FIELD_LIMITS["vehicle"]["wheelbase"].accepts(length)
                    and FIELD_LIMITS["vehicle"]["cg_to_front"].accepts(distance)
                    and 0 < distance < length
                    and FIELD_LIMITS["vehicle"]["roll_stiffness_ratio"].accepts(ratio)):
                raise ValueError
            front = 1-distance/length
            text = (f"前／後靜態重量：{front*100:.2f}%／{(1-front)*100:.2f}%\n"
                    f"KφF／KφR = {ratio:g}；固定前軸 RC 設計高度 {self.app.project['front']['rc_height']:g} m。")
        except (ValueError, ZeroDivisionError):
            text = "請確認軸距、重心位置與前後總側傾剛性比。"
        self.balance_context.configure(text="後軸 RC：解析求解零行程前後載荷轉移相等（各占 50%）。\n"+text+
                                       "\n重量比用於側向力分配；此為簡化載荷轉移模型，非實際轉向不足梯度。")

    def find_roll_center(self):
        if hardpoints.enabled(self.app.project):
            return
        if self.rc_search_dialog is not None and self.rc_search_dialog.winfo_exists():
            self.rc_search_dialog.lift()
            return
        try:
            project = read_entries(self.app.project, self.entries)
            prompt = ("後軸 RC 設計高度最低界線（m），預設 −0.005：\n"
                      "固定前軸，依解析式求解零行程前後載荷轉移量相等（各占 50%）。\n"
                      "使用前後總側傾剛性比、輪距、重心及重量比。\n"
                      "解析結果以 0.001 m 級距取整，允許上限 1 m；超界或全行程檢查不合格時不套用。\n"
                      if self.key == "rear" else
                      "RC 設計高度搜尋起點（m），預設 −0.005：\n"
                      "從此高度以 0.001 m 級距搜尋，至允許上限 1 m。\n"
                      "選擇全行程保持地面上方的最低高度。\n")
            lower = simpledialog.askfloat(
                "尋找最佳roll center",
                prompt+
                "沿用目前上下行程，RC 須全程保持地面上方。\n"
                "找到後會自動套用左側參數與最佳高度。",
                initialvalue=-.005, minvalue=FIELD_LIMITS[self.key]["rc_height"].low,
                maxvalue=FIELD_LIMITS[self.key]["rc_height"].high, parent=self)
            if lower is None:
                return
            settings = SearchSettings(round(lower*1000, 9), FIELD_LIMITS[self.key]["rc_height"].high*1000,
                                      project["analysis"]["droop_mm"], project["analysis"]["bump_mm"])
            settings.validate(require_grid=self.key != "rear")
            self.rc_search_dialog = RollCenterSearchDialog(self, project, settings)
        except (ValueError, ArithmeticError) as exc:
            messagebox.showerror("無法搜尋 RC", str(exc), parent=self)

    def find_fvsa(self):
        if hardpoints.enabled(self.app.project):
            return
        if self.fvsa_search_dialog is not None and self.fvsa_search_dialog.winfo_exists():
            self.fvsa_search_dialog.lift()
            return
        try:
            project = read_entries(self.app.project, self.entries)
            path = filedialog.askopenfilename(
                parent=self, title="尋找最佳FVSA：匯入賽道 CSV",
                initialdir=self.app.lap_path.parent if self.app.lap_path else self.app.project_dialog_dir(),
                filetypes=[("LapSimLab CSV", "*.csv")])
            if not path:
                return
            lap = read_lap_data(path)
            prepare_lap(lap)
            limit = simpledialog.askfloat(
                "尋找最佳FVSA",
                "最大允許 |Camber|（度，0～90）：\n"
                "限制全圈左右輪 Camber 的絕對值，含車身 roll／pitch，不含轉向。\n"
                f"從該軸半輪距開始，每次加長 {INPUT_STEPS['fvsa']:g} m，\n"
                "首次超限時，套用前一個合格值與目前左側參數。",
                initialvalue=2., minvalue=0., maxvalue=90., parent=self)
            if limit is None:
                return
            validate_camber_limit(limit)
            self.app.set_lap_data(lap, path)
            self.fvsa_search_dialog = FVSASearchDialog(self, project, lap, path, limit)
        except (OSError, csv.Error, ValueError, ArithmeticError) as exc:
            messagebox.showerror("無法搜尋 FVSA", str(exc), parent=self)

    def find_svsa(self):
        if self.key != "sv" or hardpoints.enabled(self.app.project):
            return
        if self.svsa_search_dialog is not None and self.svsa_search_dialog.winfo_exists():
            self.svsa_search_dialog.lift()
            return
        try:
            project = read_entries(self.app.project, self.entries)
            revision = self.app.revision
            dialog = SVSASettingsDialog(self, project, self.svsa_search_settings)
            self.app.wait_window(dialog)
            if dialog.settings is None:
                return
            if self.app.revision != revision:
                raise ValueError("輸入設定期間專案已變更，請使用目前設計重新開啟 SVSA 最佳化。")
            self.svsa_search_settings = dialog.settings
            self.svsa_search_dialog = SVSASearchDialog(self, project, dialog.settings)
        except (ValueError, ArithmeticError) as exc:
            messagebox.showerror("無法搜尋 SVSA", str(exc), parent=self)

    def find_front_caster(self):
        if self.key not in ("sv", "we") or hardpoints.enabled(self.app.project):
            return
        for panel in self.app.panels.values():
            dialog = panel.caster_search_dialog
            if dialog is not None and dialog.winfo_exists():
                dialog.lift()
                return
        try:
            project = read_entries(self.app.project, self.entries)
            revision, original_lap = self.app.revision, self.app.lap_data
            lap, path = original_lap, self.app.lap_path
            imported = lap is None or not lap.poses or not lap.front_steer_deg
            if imported:
                path = filedialog.askopenfilename(
                    parent=self, title="Caster 最佳化：匯入含車身姿態與左右前輪轉角的賽道 CSV",
                    initialdir=self.app.lap_path.parent if self.app.lap_path else self.app.project_dialog_dir(),
                    filetypes=[("LapSimLab CSV", "*.csv")])
                if not path:
                    return
                lap = read_lap_data(path)
            validate_caster_lap(lap)
            if self.app.revision != revision or self.app.lap_data is not original_lap:
                raise ValueError("選擇資料期間專案或 CSV 已變更，請使用目前資料重新搜尋。")
            if imported:
                self.app.set_lap_data(lap, path)
            self.caster_search_dialog = CasterSearchDialog(self, project, lap, path)
        except (OSError, csv.Error, ValueError, ArithmeticError) as exc:
            messagebox.showerror("無法搜尋前輪 Caster", str(exc), parent=self)

    def apply(self):
        try:
            self.app.apply_project(read_entries(self.app.project, self.entries))
        except (ValueError, ArithmeticError) as exc:
            messagebox.showerror("設定無法套用", str(exc), parent=self)

    def configure_sliders(self):
        a = self.app.project["analysis"]
        pose = self.mode.get().startswith("Heave")
        we_lap = self.key == "we" and self.plot_key == LAP_CAMBER_PLOT[0]
        labels = ["Steering (deg)"] if self.key == "we" else (["Heave (mm)", "Pitch (deg)" if self.key == "sv" else "Roll (deg)"] if pose else ["前輪上壓行程 (mm)", "後輪上壓行程 (mm)"] if self.key == "sv" else ["左輪上壓行程 (mm)", "右輪上壓行程 (mm)"])
        for i, scale in enumerate(self.sliders):
            if self.key == "we":
                low, high = -a["steer_deg"], a["steer_deg"]
            elif pose and i == 1:
                if self.key == "sv":
                    limit = self.app.engine.pitch_angle_limit()
                    low, high = -limit, limit
                else:
                    try:
                        low, high = self.app.engine.fv[self.key].roll_bounds(self.controls[0].get())
                    except Unreachable:
                        low = high = 0.
                    # Moving Heave can consume the remaining differential
                    # travel, so keep Roll inside the new per-wheel bounds.
                    value = min(high, max(low, self.controls[i].get()))
                    if value != self.controls[i].get():
                        self.controls[i].set(value)
            else:
                low, high = travel_limits(self.app.project)
            scale.configure(from_=low, to=high)
            scale.state(["disabled"] if we_lap or low == high else ["!disabled"])
            self.slider_labels[i].configure(text="前輪轉角取自 LapSim CSV" if we_lap else
                                           f"{labels[i]}  {self.controls[i].get():+.2f}")

    def change_mode(self, event=None):
        self.zero()
        self.configure_sliders()
        self.request()

    def zero(self):
        for var in self.controls:
            var.set(0.)
        self.slider_changed()

    def slider_changed(self):
        self.configure_sliders()
        if not self.ready:
            return
        self.pending_export = None
        self.ready_for_export = False
        self._slider_pending = True
        # Throttle rather than debounce: continuous movement must still solve.
        if self.timer is None:
            self.timer = self.after(FRAME_INTERVAL_MS, self.flush_slider)

    def flush_slider(self):
        self.timer = None
        if self._slider_pending and not self._request_in_flight:
            self.request()

    @property
    def plot_key(self):
        if self.plot.get() == LAP_CAMBER_PLOT[1]:
            return LAP_CAMBER_PLOT[0]
        return next(key for key, name in PLOTS[self.key] if name == self.plot.get())

    def update_lap_plots(self):
        choices = [label for _, label in PLOTS[self.key]]
        lap = self.app.lap_data
        if lap.poses and (self.key in ("front", "rear") or (self.key == "we" and lap.front_steer_deg)):
            choices.append(LAP_CAMBER_PLOT[1])
        self.plot_select.configure(values=choices)
        if self.plot.get() not in choices:
            self.plot.set(choices[0])
            self.mapping = None

    def update_camber_controls(self, show):
        if self.camber_filter is None:
            return
        if show:
            if not self.camber_filter.winfo_manager():
                self.camber_filter.pack(before=self.chart, fill="x")
        else:
            self.camber_filter.pack_forget()

    def request(self, force_mapping=False, pending_export=None):
        if not self.ready:
            return
        if self.timer:
            self.after_cancel(self.timer)
            self.timer = None
        self._slider_pending = False
        self._request_in_flight = True
        self.token += 1
        token, revision = self.token, self.app.revision
        engine, cache = self.app.engine, self.app.mapping_cache
        mode = "pose" if self.mode.get().startswith("Heave") else "travel"
        axle = self.axle.get()
        values = tuple(v.get() for v in self.controls)
        key, plot_key = self.key, self.plot_key
        need_map = force_mapping or (key != "static" and plot_key != "motion")
        lap_data = self.app.lap_data
        lap_cache = self.app.lap_cache
        lap_camber = plot_key == LAP_CAMBER_PLOT[0]
        self.update_camber_controls(lap_camber)
        we_lap = lap_camber and key == "we"
        if key == "we":
            if we_lap:
                self.axle_controls.pack_forget()
                axle = "front"
            elif not self.axle_controls.winfo_manager():
                self.axle_controls.pack(side="left")
            self.zero_button.state(["disabled"] if we_lap else ["!disabled"])
            self.configure_sliders()
        need_lap = (lap_data and self.app.show_lap.get() and key in LAP_PAGES
                    and plot_key != "motion" and not lap_camber)
        self.pending_export = pending_export
        self.ready_for_export = False
        self.app.status.set(f"{PAGES[key]} · 計算中…")
        cancelled = lambda: (self.token != token or self.app.revision != revision or self.app.closing
                             or (lap_camber and self.app.lap_data is not lap_data))
        def work():
            if cancelled():
                return
            state, error, mapping = None, None, None
            try:
                try:
                    if key in ("front", "rear"):
                        state = engine.fv[key].pose(*values) if mode == "pose" else engine.fv[key].solve(*values)
                    elif key == "sv":
                        state = engine.sv_pose(*values) if mode == "pose" else engine.solve_sv(*values)
                    elif key == "we":
                        state = engine.solve_we(values[0], axle)
                    else:
                        state = {"metrics": {"wireframe_lines": len(engine.scene.lines),
                                             "named_points": len(static_point_rows(engine)),
                                             **engine.static_anti_metrics}, "points": {}}
                except ValueError as exc:
                    error = str(exc)
                if lap_camber and not cancelled():
                    camber_key = (key, plot_key)
                    mapping = lap_cache.get(camber_key)
                    if mapping is None:
                        mapping = calculate_lap_camber(engine, "front" if we_lap else key, lap_data, cancelled,
                                                       include_steering=we_lap)
                        if mapping is not None and not cancelled():
                            lap_cache[camber_key] = mapping
                elif need_map and not cancelled():
                    cache_key = (key, mode, axle)
                    mapping = cache.get(cache_key)
                    if mapping is None:
                        mapping = calculate_mapping(engine, key, mode, axle, cancelled)
                        if mapping is not None and not cancelled():
                            cache[cache_key] = mapping
                if need_lap and key not in lap_cache and not cancelled():
                    lap_cancelled = lambda: (self.app.revision != revision or self.app.closing
                                             or self.app.lap_data is not lap_data)
                    lap = calculate_lap_metrics(engine, key, lap_data.travels, lap_cancelled)
                    if lap is not None and not lap_cancelled():
                        lap_cache[key] = lap
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"
            if not cancelled():
                self.app.results.put((self, token, revision, state, mapping, error))
        self.app.jobs.put(work)

    def receive(self, state, mapping, error):
        self._request_in_flight = False
        self.state, self.mapping, self.error = state, mapping, error
        self.ready_for_export = True
        self.metrics.delete(*self.metrics.get_children())
        if self.plot_key == LAP_CAMBER_PLOT[0] and mapping:
            self.metrics.heading("metric", text="圈速過彎 Camber")
            self.metrics.heading("value", text="樣本數／最小至最大值（deg）")
            self.metrics.insert("", "end", values=("有效彎道／全部彎道",
                                f"{mapping['valid']:,} / {mapping['total']:,}（排除直線 {mapping['straight']:,} 筆）"))
            for key, label, _, _ in LAP_CAMBER_SERIES_BY_PAGE[self.key]:
                values = [row[key] for row in mapping["rows"]]
                self.metrics.insert("", "end", values=(label, f"{min(values):+.3f} ～ {max(values):+.3f}" if values else "N/A"))
        else:
            self.metrics.heading("metric", text="目前姿態結果 / 單位")
            self.metrics.heading("value", text="數值（N/A：目前姿態無有效定義）")
            for key, value in (state or {}).get("metrics", {}).items():
                self.metrics.insert("", "end", values=(RESULT_LABELS.get(key, key), "N/A" if value is None else f"{value:.7g}"))
        self.render()
        samples = "彎道" if self.plot_key == LAP_CAMBER_PLOT[0] else "掃描"
        coverage = f" · {samples} {mapping['valid']}/{mapping['total']} 有效" if mapping else ""
        self.app.status.set((f"{PAGES[self.key]} · {error}" if error else f"{PAGES[self.key]} · 完成")+coverage)
        if self.pending_export:
            kind, path = self.pending_export
            self.pending_export = None
            if mapping:
                self.app.perform_export(kind, path, self)
        if self._slider_pending:
            # Display completed work even while the pointer moves, then solve
            # the latest slider values. Keep at most one pose solve in flight.
            self.ready_for_export = False
            if self.timer is None:
                self.timer = self.after(FRAME_INTERVAL_MS, self.flush_slider)

    def render(self):
        if not self.ready:
            return
        plot = self.plot_key
        lap_camber = plot == LAP_CAMBER_PLOT[0]
        self.update_camber_controls(lap_camber)
        if self.key != "static" and plot == "motion" and not self.state:
            self.chart.set(None, preserve_view=True)
            return
        mapping = self.mapping
        if lap_camber:
            mapping = self.app.lap_cache.get((self.key, plot))
        elif mapping and "grid" not in mapping:
            mapping = None
        spec = make_plot(self.key, plot, self.state, mapping, self.app.engine,
                         {k: v.get() for k, v in self.groups.items()},
                         show_static_references=self.static_references.get(),
                         lap=self.app.lap_cache.get(self.key) if self.app.show_lap.get() else None)
        if spec and lap_camber:
            spec["show_normal_curves"] = self.show_normal_curves.get()
            spec["hidden_series"] = [label for metric, label, _, _ in LAP_CAMBER_SERIES_BY_PAGE[self.key]
                                     if not self.camber_visible[metric].get()]
        if spec and "world_lines" in spec and self.key != "static" and not self.construction.get():
            spec["world_lines"] = [p for p in spec["world_lines"] if not p["construction"]]
        source = id(self.app.lap_data) if lap_camber else None
        chart_key = (plot, self.mode.get(), self.axle.get(), source)
        self.chart.set(spec, preserve_view=chart_key == self._chart_key)
        self._chart_key = chart_key


class App(tk.Tk):
    def __init__(self, project=None):
        super().__init__()
        self.title("SuspensionGeometryLab · 懸吊幾何工作台")
        self.geometry("1500x960")
        self.minsize(1150, 760)
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TFrame", background="#f1f5f9")
        style.configure("TLabel", background="#f1f5f9", font=("Microsoft JhengHei UI", 10))
        style.configure("TButton", padding=(9, 5), font=("Microsoft JhengHei UI", 10))
        style.configure("RCSearch.TButton", padding=(4, 3), font=("Microsoft JhengHei UI", 9))
        style.configure("TNotebook.Tab", padding=(22, 9), font=("Microsoft JhengHei UI", 11))
        style.configure("TEntry", fieldbackground="#ffffff", foreground="#0f172a")
        style.configure("Invalid.TEntry", fieldbackground="#fff7ed", foreground="#9a3412")
        style.map("TEntry",
                  fieldbackground=[("disabled", "#cbd5e1"), ("readonly", "#e2e8f0")],
                  foreground=[("disabled", "#475569"), ("readonly", "#475569")])
        self.project = copy.deepcopy(validate(project or default_project()))
        self.engine = Engine(self.project)
        check_mount_inputs(self.engine)
        self.revision = 0
        self.dirty = self.closing = False
        self.step_options = {"include_tires": True, "include_static_camber": True,
                             "export_solids": False, "tube_diameter_mm": 20.}
        self.path = None
        self.editor = None
        self.mapping_cache = {}
        self.lap_cache = {}
        self.lap_data = None
        self.lap_path = None
        self.show_lap = tk.BooleanVar(value=False)
        self.lap_source = tk.StringVar(value="尚未匯入")
        self.jobs, self.results = queue.Queue(), queue.Queue()
        header = ttk.Frame(self, padding=(16, 12))
        header.pack(fill="x")
        ttk.Label(header, text="SUSPENSION GEOMETRY LAB", font=("Segoe UI", 19, "bold")).pack(side="left")
        self.project_label = ttk.Label(header, text=self.project["name"], foreground="#475569")
        self.project_label.pack(side="left", padx=20)
        for text, command in (("共用車輛／懸吊設定", self.open_parameters), ("匯入", self.open_project),
                              ("儲存專案", self.save), ("另存專案", lambda: self.save(True))):
            ttk.Button(header, text=text, command=command).pack(side="right", padx=4)
        self.notebook = ttk.Notebook(self)
        self.panels = {key: Panel(self, key) for key in PAGES}
        for key, panel in self.panels.items():
            self.notebook.add(panel, text=PAGES[key])
        self.tire_panel = TirePanel(self)
        self.notebook.add(self.tire_panel, text="輪胎模型 · Pacejka")
        footer = ttk.Frame(self, padding=(12, 8))
        footer.pack(side="bottom", fill="x")
        self.export_buttons = {}
        for kind, label in (("state", "姿態 CSV"), ("points", "硬點 CSV"), ("mapping", "範圍 CSV"),
                            ("png", "PNG 圖片"), ("svg", "SVG 向量圖"), ("report", "HTML 報告 / PDF")):
            button = self.export_buttons[kind] = ttk.Button(footer, text=label, command=lambda k=kind: self.export(k))
            button.pack(side="left", padx=3)
        self.step_button = ttk.Button(footer, text="STEP 匯出", command=lambda: self.export("step"))
        self.step_button.pack(side="left", padx=3)
        self.notebook.bind("<<NotebookTabChanged>>", self.change_page)
        self.status = tk.StringVar(value="靜態幾何已建立")
        ttk.Label(self, textvariable=self.status, padding=(16, 4), foreground="#475569").pack(side="bottom", fill="x")
        self.notebook.pack(fill="both", expand=True, padx=12)
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.bind("<Control-s>", lambda e: self.save())
        self.bind("<Control-o>", lambda e: self.open_project())
        self.worker_thread = threading.Thread(target=self.worker, daemon=True)
        self.worker_thread.start()
        self.poll_timer = self.after(FRAME_INTERVAL_MS, self.poll)
        self.after_idle(self.active.request)

    @property
    def active(self):
        return self.nametowidget(self.notebook.select())

    def change_page(self, _event):
        panel = self.active
        self.step_button.state(["!disabled" if panel.key in STEP_PAGES else "disabled"])
        for kind in ("points", "report"):
            self.export_buttons[kind].state(["disabled" if panel is self.tire_panel else "!disabled"])
        panel.request()

    def worker(self):
        while True:
            job = self.jobs.get()
            if job is None:
                return
            job()

    def poll(self):
        try:
            while True:
                panel, token, revision, state, mapping, error = self.results.get_nowait()
                if token == panel.token and revision == self.revision:
                    panel.receive(state, mapping, error)
        except queue.Empty:
            pass
        if not self.closing:
            self.poll_timer = self.after(FRAME_INTERVAL_MS, self.poll)

    def apply_project(self, candidate, mark_dirty=True, preserve_pose=True):
        # Build everything before committing, so invalid geometry is transactional.
        engine = Engine(candidate)
        check_mount_inputs(engine)
        settings_changed = any(self.project["vehicle"][key] != engine.project["vehicle"][key] for key in NON_KINEMATIC_VEHICLE_FIELDS)
        keep_pose = (preserve_pose and mark_dirty and settings_changed
                     and pose_settings(self.project) == pose_settings(engine.project))
        self.project, self.engine = copy.deepcopy(engine.project), engine
        self.revision += 1
        self.mapping_cache = {}
        self.lap_cache = {}
        self.dirty = mark_dirty
        self.project_label.configure(text=self.project["name"]+(" *" if self.dirty else ""))
        for panel in self.panels.values():
            panel.sync(keep_pose)
        self.active.request()

    def open_parameters(self):
        if self.editor is not None and self.editor.winfo_exists():
            self.editor.lift()
        else:
            self.editor = ParameterEditor(self)

    def project_dialog_dir(self):
        if self.path is not None:
            return self.path.parent.resolve()
        return PROJECTS if PROJECTS.is_dir() else ROOT

    def save(self, save_as=False):
        path = None if save_as else self.path
        if path is None:
            path = filedialog.asksaveasfilename(
                parent=self, title="儲存 SuspensionGeometryLab 專案",
                initialdir=self.project_dialog_dir(),
                initialfile=self.path.name if self.path is not None else "suspension_geometry.json",
                defaultextension=".json", filetypes=[("JSON project", "*.json")],
            )
        if not path:
            return False
        try:
            save_project(path, self.project)
            self.path, self.dirty = Path(path), False
            self.project_label.configure(text=self.project["name"])
            self.status.set(f"專案已儲存：{path}")
            return True
        except (OSError, ValueError) as exc:
            messagebox.showerror("儲存失敗", str(exc), parent=self)
            return False

    def confirm_unsaved(self):
        if not self.dirty:
            return True
        answer = messagebox.askyesnocancel("尚未儲存", "要先儲存目前專案嗎？", parent=self)
        return False if answer is None else self.save() if answer else True

    def open_project(self):
        if not self.confirm_unsaved():
            return
        path = filedialog.askopenfilename(
            parent=self, title="開啟 SuspensionGeometryLab 專案",
            initialdir=self.project_dialog_dir(), filetypes=[("JSON project", "*.json")],
        )
        if path:
            self.import_project(path)

    def open_lap(self):
        path = filedialog.askopenfilename(
            parent=self, title="開啟 LapSimLab lap_result.csv",
            initialdir=self.lap_path.parent if self.lap_path else self.project_dialog_dir(),
            filetypes=[("LapSimLab CSV", "*.csv")],
        )
        if path:
            self.import_lap(path)

    def import_lap(self, path):
        try:
            lap_data = read_lap_data(path)
        except (OSError, ValueError, csv.Error) as exc:
            messagebox.showerror("無法匯入 LapSim CSV", str(exc), parent=self)
            return
        self.set_lap_data(lap_data, path)

    def set_lap_data(self, lap_data, path):
        was_camber = self.active.plot_key == LAP_CAMBER_PLOT[0]
        self.lap_data = lap_data
        self.lap_path = Path(path)
        self.lap_cache = {}
        self.show_lap.set(True)
        count = len(lap_data.travels)
        self.lap_source.set(f"{self.lap_path.name} · {count:,} 筆")
        for key in LAP_PAGES:
            self.panels[key].lap_check.state(["!disabled"])
            self.panels[key].update_lap_plots()
        self.panels["we"].update_lap_plots()
        self.panels["we"].render()
        self.status.set(f"已匯入 {self.lap_path.name} · {count:,} 筆")
        if not lap_data.poses:
            self.status.set(self.status.get() + " · 缺少車身姿態或轉彎方向，僅供行程疊圖")
        elif self.active.key == "we" and not lap_data.front_steer_deg:
            self.status.set(self.status.get() + " · 缺少左右前輪轉角，無法產生輪端圈速 Camber 圖")
        self.toggle_lap()
        if was_camber:
            self.active.request()

    def toggle_lap(self):
        for key in LAP_PAGES:
            self.panels[key].render()
        panel = self.active
        if (self.show_lap.get() and panel.key in LAP_PAGES and panel.plot_key not in ("motion", LAP_CAMBER_PLOT[0])
                and panel.key not in self.lap_cache):
            panel.request()

    def import_project(self, path):
        try:
            project, missing = read_import_project(path)
            if missing:
                dialog = ImportRepairDialog(self, project, missing)
                self.wait_window(dialog)
                if dialog.applied:
                    self.path = None  # Prompt for a save location for the repaired copy.
            else:
                self.apply_project(project, False)
                self.path = Path(path)
        except (OSError, ValueError, ArithmeticError) as exc:
            messagebox.showerror("無法開啟專案", str(exc), parent=self)

    def export(self, kind):
        panel = self.active
        if panel is self.tire_panel and kind in ("points", "report"):
            return
        if kind == "step" and panel.key not in STEP_PAGES:
            self.status.set("STEP 匯出適用於前 FV、後 FV、SV 與靜態全車。")
            return
        if kind == "mapping" and panel.key == "static":
            self.status.set("靜態全車請使用硬點 CSV；範圍掃描在 FV / SV / WE3D 頁。")
            return
        if not getattr(panel, "ready_for_export", False):
            self.status.set("請先計算有效的輪胎工況。" if panel is self.tire_panel else
                            "正在計算目前姿態，完成後即可匯出。")
            return
        if kind in ("state", "points", "step") and not panel.state:
            self.status.set("目前姿態無解；請調整滑桿後再匯出。")
            return
        if kind in ("png", "svg", "report") and panel.chart.spec is None:
            self.status.set("目前沒有可匯出的圖表。")
            return
        if kind == "step":
            dialog = StepExportDialog(self, self.step_options)
            self.wait_window(dialog)
            if dialog.options is None:
                return
            self.step_options = dialog.options
        extension = {"state": ".csv", "points": ".csv", "mapping": ".csv", "png": ".png",
                     "svg": ".svg", "report": ".html", "step": ".step"}[kind]
        name = "lap_camber" if kind == "mapping" and panel.plot_key == LAP_CAMBER_PLOT[0] else kind
        path = filedialog.asksaveasfilename(parent=self, defaultextension=extension,
                    initialfile=f"suspension_{panel.key}_{name}{extension}", filetypes=[(extension, "*"+extension)])
        if not path:
            return
        if kind == "mapping" and panel.mapping is None:
            panel.request(force_mapping=True, pending_export=(kind, path))
        else:
            if kind == "step":
                self.perform_export(kind, path, panel, step_options=self.step_options)
            else:
                self.perform_export(kind, path, panel)

    def perform_export(self, kind, path, panel, *, step_options=None):
        try:
            if kind == "mapping":
                write_csv(path, panel.mapping["rows"])
            elif kind in ("state", "points"):
                rows = static_point_rows(self.engine) if panel.key == "static" else point_rows(panel.state) if kind == "points" else [panel.state["metrics"]]
                write_csv(path, rows)
            elif kind == "png":
                panel.chart.png(path)
            elif kind == "svg":
                Path(path).write_text(panel.chart.svg(), encoding="utf-8")
            elif kind == "step":
                write_step(path, self.engine, panel.key, panel.state, show_construction=panel.construction.get(),
                           **(self.step_options if step_options is None else step_options))
            else:
                controls = {"sliders": [v.get() for v in panel.controls], "mode": panel.mode.get(), "axle": panel.axle.get(), "plot": panel.plot_key}
                if panel.key == "static":
                    controls.update(static_references=panel.static_references.get(),
                                    static_groups={k: v.get() for k, v in panel.groups.items()})
                if panel.plot_key == LAP_CAMBER_PLOT[0]:
                    controls["lap_csv"] = self.lap_path.name
                    controls["lap_visible_series"] = [metric for metric, var in panel.camber_visible.items() if var.get()]
                    controls["lap_normal_curves"] = panel.show_normal_curves.get()
                    if panel.key == "we":
                        controls["axle"] = "front"
                write_report(path, self.project, panel.key, panel.state, panel.mapping, panel.chart.svg(), controls,
                             derived_parameters=self.engine.derived_parameters,
                             mount_position_checks=self.engine.scene.geometry.get("mount_position_checks"))
            self.status.set(f"已匯出：{path}")
        except (OSError, ValueError) as exc:
            messagebox.showerror("匯出失敗", str(exc), parent=self)

    def close(self):
        if not self.confirm_unsaved():
            return
        self.closing = True
        for panel in self.panels.values():
            panel.token += 1
            if panel.timer:
                panel.after_cancel(panel.timer)
                panel.timer = None
        self.after_cancel(self.poll_timer)
        self.jobs.put(None)
        self.destroy()


def main():
    import argparse
    parser = argparse.ArgumentParser(description="SuspensionGeometryLab native geometry workbench")
    parser.add_argument("--project", type=Path, help="Open a SuspensionGeometryLab JSON project")
    args = parser.parse_args()
    app = App()
    if args.project:
        app.wait_visibility()
        app.after_idle(lambda: app.import_project(args.project))
    app.mainloop()
