"""SVSA setup and cancellable, transactional search progress."""
import queue
import threading
import tkinter as tk
from tkinter import ttk

from .roll_center_search import SearchCancelled
from .svsa_search import SVSA_MAX_M, SVSA_STEP_M, SVSASearchSettings, search_svsa, svsa_candidates


AXLE_LABELS = {"front": "前軸", "rear": "後軸"}
BOUND_SYMBOLS = {"min": "≥", "max": "≤"}
CANDIDATE_RANGE = (f"前軸從 {svsa_candidates('front')[0]:g} m、後軸從 {svsa_candidates('rear')[0]:g} m 起，"
                   f"以 {SVSA_STEP_M:g} m 級距搜尋至 {SVSA_MAX_M:g} m。")


def _show_modal(dialog, app):
    dialog.update_idletasks()
    x = max(0, app.winfo_rootx()+(app.winfo_width()-dialog.winfo_reqwidth())//2)
    y = max(0, app.winfo_rooty()+(app.winfo_height()-dialog.winfo_reqheight())//2)
    dialog.geometry(f"+{x}+{y}")
    dialog.deiconify()
    dialog.grab_set()


class SVSASettingsDialog(tk.Toplevel):
    def __init__(self, panel, project, defaults):
        super().__init__(panel.app)
        self.withdraw()
        self.project, self.settings = project, None
        self.title("SVSA 最佳化設定")
        self.transient(panel.app)
        self.resizable(False, False)
        self.bind("<Escape>", lambda event: self.destroy())
        self.bind("<Return>", lambda event: self.accept())
        self.protocol("WM_DELETE_WINDOW", self.destroy)
        body = ttk.Frame(self, padding=18)
        body.pack(fill="both", expand=True)
        ttk.Label(body, text="前後輪 SVSA 最佳化", font=("Microsoft JhengHei UI", 11, "bold")).pack(anchor="w")
        self.inputs, self.entries, self.bounds = {}, {}, {}
        row = ttk.Frame(body)
        row.pack(fill="x", pady=(12, 0))
        ttk.Label(row, text="目標 Heave（mm，正值上壓）").pack(side="left")
        self.inputs["heave_mm"] = tk.StringVar(self, value=f"{defaults.heave_mm:g}")
        self.entries["heave_mm"] = ttk.Entry(row, textvariable=self.inputs["heave_mm"], width=14)
        self.entries["heave_mm"].pack(side="right", padx=(18, 0))
        for axle, label in AXLE_LABELS.items():
            bound, threshold = defaults.criterion(axle)
            row = ttk.LabelFrame(body, text=f"{label} Caster 絕對變化量", padding=(10, 7))
            row.pack(fill="x", pady=(12, 0))
            self.bounds[axle] = tk.StringVar(self, value=bound)
            for value, choice in (("min", "下限（≥）"), ("max", "上限（≤）")):
                ttk.Radiobutton(row, text=choice, variable=self.bounds[axle], value=value).pack(
                    side="left", padx=(0, 12))
            key = f"{axle}_limit_deg"
            self.inputs[key] = tk.StringVar(self, value=f"{threshold:g}")
            self.entries[key] = ttk.Entry(row, textvariable=self.inputs[key], width=14)
            self.entries[key].pack(side="right")
            ttk.Label(row, text="門檻（°）").pack(side="right", padx=(12, 6))
        analysis = project["analysis"]
        ttk.Label(body, text=(f"Heave 沿用目前行程：−{analysis['droop_mm']:g}～+{analysis['bump_mm']:g} mm。\n"
                  "比較零行程與目標 Heave 的 Caster 絕對變化量（Pitch＝0）。\n"
                  f"{CANDIDATE_RANGE}\n"
                  "選下限時，在第一個不合格或無解的候選之前取最長值。\n"
                  "選上限時，取第一個合格的最短值，可略過不合格或無解的候選。\n"
                  "兩軸皆合格且組合姿態通過驗證後，才一併套用結果及左側參數。"),
                  wraplength=540, foreground="#64748b").pack(fill="x", pady=(14, 4))
        self.error = ttk.Label(body, text="", foreground="#b45309", wraplength=540)
        self.error.pack(fill="x", pady=(4, 12))
        buttons = ttk.Frame(body)
        buttons.pack(fill="x")
        ttk.Button(buttons, text="開始搜尋", command=self.accept).pack(side="right")
        ttk.Button(buttons, text="取消", command=self.destroy).pack(side="right", padx=(0, 8))
        _show_modal(self, panel.app)

    def accept(self):
        try:
            values = {key: float(var.get()) for key, var in self.inputs.items()}
        except ValueError:
            self.error.configure(text="三個欄位都需要數值，請確認沒有空白或文字。")
            return
        try:
            settings = SVSASearchSettings.from_bounds(
                values["heave_mm"], self.bounds["front"].get(), values["front_limit_deg"],
                self.bounds["rear"].get(), values["rear_limit_deg"],
            )
            settings.validate(self.project)
        except (ValueError, ArithmeticError) as exc:
            self.error.configure(text=f"請檢查輸入：{exc}")
            return
        self.settings = settings
        self.destroy()

    def destroy(self):
        super().destroy()
        # Finalize Tk variables on the UI thread, not on a solver's GC pass.
        self.inputs.clear()
        self.entries.clear()
        self.bounds.clear()
        self.error = None


class SVSASearchDialog(tk.Toplevel):
    def __init__(self, panel, project, settings):
        super().__init__(panel.app)
        self.withdraw()
        self.panel, self.app, self.settings = panel, panel.app, settings
        self.revision = self.app.revision
        self.cancel_event = threading.Event()
        self.messages = queue.Queue()
        self.poll_timer = None
        self.title("尋找最佳SVSA")
        self.transient(self.app)
        self.resizable(False, False)
        self.protocol("WM_DELETE_WINDOW", self.destroy)
        self.bind("<Escape>", lambda event: self.destroy())
        body = ttk.Frame(self, padding=18)
        body.pack(fill="both", expand=True)
        ttk.Label(body, text="尋找符合 Caster 變化條件的 SVSA",
                  font=("Microsoft JhengHei UI", 11, "bold")).pack(anchor="w")
        criteria = "\n".join(f"{label} |ΔCaster| {self._criterion(axle)}"
                              for axle, label in AXLE_LABELS.items())
        ttk.Label(body, text=(f"Heave {settings.heave_mm:+g} mm，Pitch＝0\n{criteria}\n{CANDIDATE_RANGE}"),
                  wraplength=560).pack(fill="x", pady=(12, 8))
        self.progress = ttk.Progressbar(body, length=560, maximum=100)
        self.progress.pack(fill="x")
        self.status = tk.StringVar(value=f"正在檢查前軸 SVSA {svsa_candidates('front')[0]:g} m…")
        ttk.Label(body, textvariable=self.status, wraplength=560).pack(fill="x", pady=12)
        ttk.Label(body, text="使用 SV 平面固定切線模型，只比較零行程與目標 Heave 兩個姿態；沿用左側的 Anti 與初始 Caster。",
                  wraplength=560, foreground="#64748b").pack(fill="x")
        self.close_button = ttk.Button(body, text="取消搜尋", command=self.destroy)
        self.close_button.pack(anchor="e", pady=(12, 0))
        _show_modal(self, self.app)
        self.thread = threading.Thread(target=self.run_search,
                                       args=(project, settings, self.cancel_event, self.messages), daemon=True)
        self.thread.start()
        self.poll_timer = self.after(40, self.poll)

    @staticmethod
    def run_search(project, settings, cancel_event, messages):
        try:
            result = search_svsa(project, settings, cancelled=cancel_event.is_set,
                                 progress=lambda *value: messages.put(("progress", value)))
            messages.put(("result", result))
        except SearchCancelled:
            pass
        except Exception as exc:
            messages.put(("error", str(exc)))

    def poll(self):
        self.poll_timer = None
        finished = False
        try:
            while True:
                kind, value = self.messages.get_nowait()
                if kind == "progress":
                    axle, tested, total, length, change, reason = value
                    self.progress.configure(value=(50 if axle == "rear" else 0)+tested/total*50)
                    angle = (f"|ΔCaster| {change.absolute_deg:.6f}°，條件 {self._criterion(axle)} · {reason}"
                             if change is not None else reason)
                    self.status.set(f"{AXLE_LABELS[axle]} SVSA {length:g} m，"
                                    f"已檢查 {tested}/{total} 個長度\n{angle}")
                elif kind == "error":
                    self.status.set(f"搜尋失敗：{value}\n原設計未變更。")
                    finished = True
                else:
                    self.finish(value)
                    finished = True
        except queue.Empty:
            pass
        if finished:
            self.progress.configure(value=100)
            self.close_button.configure(text="關閉")
        else:
            self.poll_timer = self.after(40, self.poll)

    def _criterion(self, axle):
        bound, threshold = self.settings.criterion(axle)
        return f"{BOUND_SYMBOLS[bound]} {threshold:g}°"

    def _summary(self, axle, result):
        title = AXLE_LABELS[axle]
        criterion = self._criterion(axle)
        if result.best is None:
            text = (f"{title}尋找 |ΔCaster| {criterion} 的 SVSA，"
                    f"已檢查 {result.tested} 個長度，沒有合格結果。")
        else:
            change = result.best
            text = (f"{title}最佳 SVSA：{change.svsa_m:g} m\n"
                    f"Caster {change.zero_deg:+.6f}° → {change.target_deg:+.6f}°，"
                    f"Δ {change.signed_deg:+.6f}°，|Δ| {change.absolute_deg:.6f}° {criterion}")
        if result.stop_reason == "caster_limit":
            label = "下一級" if result.best is not None else "起始候選"
            text += (f"\n{label} {result.stopped_svsa_m:g} m 的 |ΔCaster| "
                     f"{result.stopped_change.absolute_deg:.6f}° 未達{title}下限，停止搜尋。")
        elif result.stop_reason == "geometry":
            text += f"\n在 {result.stopped_svsa_m:g} m 因幾何／姿態無解停止：{result.error}"
        elif result.stop_reason == "upper_limit":
            text += f"\n已達 {SVSA_MAX_M:g} m，採範圍內最長值。"
        elif result.stop_reason == "found":
            text += (f"\n從 {svsa_candidates(axle)[0]:g} m 起首次合格，"
                     f"為最短值。已檢查 {result.tested} 個長度。")
        elif result.error:
            text += f"\n最近的幾何失敗原因：{result.error}"
        return text

    def finish(self, result):
        summary = self._summary("front", result.front)+"\n\n"+self._summary("rear", result.rear)
        if result.project is None:
            self.status.set(f"{summary}\n\n未套用：{result.error}\n原設計未變更。")
            return
        if self.app.revision != self.revision:
            self.status.set("搜尋期間專案已變更，結果未套用；請使用目前設計重新搜尋。")
            return
        try:
            self.app.apply_project(result.project)
        except (ValueError, ArithmeticError) as exc:
            self.status.set(f"搜尋結果無法套用：{exc}")
            return
        # Show the tested target pose in the existing SV table and diagram.
        pose = self.panel.mode.get().startswith("Heave")
        self.panel.controls[0].set(self.settings.heave_mm)
        self.panel.controls[1].set(0. if pose else self.settings.heave_mm)
        self.panel.configure_sliders()
        self.panel.request()
        self.status.set("已同時套用前後輪最佳 SVSA\n\n"+summary)
        self.app.status.set(f"已套用 SVSA：前 {result.front.best.svsa_m:g} m／後 {result.rear.best.svsa_m:g} m")

    def destroy(self):
        self.cancel_event.set()
        if self.poll_timer is not None:
            self.after_cancel(self.poll_timer)
            self.poll_timer = None
        super().destroy()
        self.status = None
        self.progress = None
        self.close_button = None
