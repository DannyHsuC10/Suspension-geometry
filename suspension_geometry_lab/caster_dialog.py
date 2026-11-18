"""Cancellable front-caster search with a normal-fit result preview."""
from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import ttk

from .analysis import LAP_CAMBER_PLOT
from .caster_search import CASTER_MAX_DEG, CASTER_MIN_DEG, OUTER_STEERED_CAMBER, search_front_caster
from .input_limits import INPUT_STEPS
from .roll_center_search import SearchCancelled


class CasterSearchDialog(tk.Toplevel):
    def __init__(self, panel, project, lap, path):
        super().__init__(panel.app)
        self.withdraw()
        self.panel, self.app, self.lap = panel, panel.app, lap
        self.revision = self.app.revision
        self.cancel_event = threading.Event()
        self.messages = queue.Queue()
        self.poll_timer = None
        self.title("尋找最佳前輪 Caster")
        self.transient(self.app)
        self.resizable(False, False)
        self.protocol("WM_DELETE_WINDOW", self.destroy)
        self.bind("<Escape>", lambda event: self.destroy())
        body = ttk.Frame(self, padding=18)
        body.pack(fill="both", expand=True)
        ttk.Label(body, text="前輪 Caster · 外側輪常態分布峰值最接近 0°",
                  font=("Microsoft JhengHei UI", 11, "bold")).pack(anchor="w")
        name = Path(path).name if path else "目前載入的 CSV"
        ttk.Label(body, text=(f"CSV：{name}\n"
                              f"搜尋 {CASTER_MIN_DEG:g}～{CASTER_MAX_DEG:g}°，每次 {INPUT_STEPS['caster_deg']:g}°。\n"
                              "使用外側輪「姿態＋懸吊＋轉向」Camber，包含靜態 Camber。\n"
                              "左右轉合併、排除直線，依目前圖表的有效彎道樣本擬合。"),
                  wraplength=560).pack(fill="x", pady=(12, 8))
        self.progress = ttk.Progressbar(body, length=560, maximum=100)
        self.progress.pack(fill="x")
        self.status = tk.StringVar(value="正在計算目前 Caster 的外側輪 Camber 分布…")
        ttk.Label(body, textvariable=self.status, wraplength=560).pack(fill="x", pady=12)
        ttk.Label(body, text="常態曲線峰值位於平均值 μ，目標是最小化 |μ|；不另外限制標準差或最大 Camber。\n"
                  "只調整前輪設計 Caster，固定 CSV 的行程、車身姿態與轉角，不重新模擬圈速。",
                  wraplength=560, foreground="#64748b").pack(fill="x")
        self.close_button = ttk.Button(body, text="取消搜尋", command=self.destroy)
        self.close_button.pack(anchor="e", pady=(12, 0))
        self.update_idletasks()
        x = max(0, self.app.winfo_rootx()+(self.app.winfo_width()-self.winfo_reqwidth())//2)
        y = max(0, self.app.winfo_rooty()+(self.app.winfo_height()-self.winfo_reqheight())//2)
        self.geometry(f"+{x}+{y}")
        self.deiconify()
        self.grab_set()
        self.thread = threading.Thread(target=self.run_search,
                                       args=(project, lap, self.cancel_event, self.messages), daemon=True)
        self.thread.start()
        self.poll_timer = self.after(40, self.poll)

    @staticmethod
    def run_search(project, lap, cancel_event, messages):
        try:
            result = search_front_caster(project, lap, cancelled=cancel_event.is_set,
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
                    tested, total, caster, fit, reason = value
                    self.progress.configure(value=tested/total*100)
                    detail = f"峰值 μ {fit.mean_deg:+.6f}°，σ {fit.std_deg:.6f}°" if fit is not None else reason
                    self.status.set(f"已檢查 {tested}/{total} 個角度；Caster {caster:g}°\n{detail}")
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

    def finish(self, result):
        if result.project is None:
            reasons = "\n".join(f"{reason}：{count} 個" for reason, count in
                                sorted(result.rejected.items(), key=lambda pair: -pair[1])[:3])
            self.status.set(f"3～6° 範圍內沒有可套用的常態分布結果。\n{reasons}\n原設計未變更。")
            return
        if self.app.revision != self.revision or self.app.lap_data is not self.lap:
            self.status.set("搜尋期間專案或 CSV 已變更，結果未套用；請使用目前資料重新搜尋。")
            return
        try:
            self.app.apply_project(result.project)
        except (ValueError, ArithmeticError) as exc:
            self.status.set(f"搜尋結果無法套用：{exc}")
            return
        # Reuse the exact winning samples, making the plotted peak reviewable.
        self.app.lap_cache[("we", LAP_CAMBER_PLOT[0])] = result.mapping
        target = self.app.panels["we"]
        target.axle.set("front")
        target.plot.set(LAP_CAMBER_PLOT[1])
        target.show_normal_curves.set(True)
        target.camber_visible[OUTER_STEERED_CAMBER].set(True)
        self.app.notebook.select(target)
        target.request()
        fit = result.best
        before = (f"搜尋前 Caster {result.before.caster_deg:g}°：峰值 {result.before.mean_deg:+.6f}°\n"
                  if result.before is not None else "")
        boundary = "最佳值位於搜尋界線，結果僅限 3～6° 範圍。\n" if fit.caster_deg in (CASTER_MIN_DEG, CASTER_MAX_DEG) else ""
        self.status.set(f"已套用最佳前輪 Caster：{fit.caster_deg:g}°\n"+before+
                        f"最佳常態分布峰值 μ：{fit.mean_deg:+.6f}°（距離 0°：{fit.error_deg:.6f}°）\n"
                        f"標準差 σ：{fit.std_deg:.6f}°\n"
                        f"外側輪 Camber 範圍：{fit.minimum_deg:+.6f}～{fit.maximum_deg:+.6f}°\n"
                        f"有效彎道：{fit.samples:,}/{result.mapping['total']:,} 筆；"
                        f"排除直線 {result.mapping['straight']:,} 筆。\n"
                        f"檢查 {result.tested} 個角度；所有候選使用相同有效樣本。\n"+boundary+
                        "已顯示輪端的過彎 Camber 分布與常態曲線。")
        self.app.status.set(f"已套用前輪 Caster {fit.caster_deg:g}°；外側輪常態峰值 {fit.mean_deg:+.6f}°")

    def destroy(self):
        self.cancel_event.set()
        if self.poll_timer is not None:
            self.after_cancel(self.poll_timer)
            self.poll_timer = None
        super().destroy()
        self.status = None
        self.progress = None
        self.close_button = None
