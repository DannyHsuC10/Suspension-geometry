"""Modal lap-based FVSA search with cancellation and transactional apply."""
from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import ttk

from .fvsa_search import search_fvsa
from .input_limits import AXLE_LIMITS, INPUT_STEPS
from .roll_center_search import SearchCancelled


class FVSASearchDialog(tk.Toplevel):
    def __init__(self, panel, project, lap, path, max_camber_deg):
        super().__init__(panel.app)
        self.withdraw()
        self.panel, self.app = panel, panel.app
        self.revision = self.app.revision
        self.max_camber_deg = max_camber_deg
        self.cancel_event = threading.Event()
        self.messages = queue.Queue()
        self.poll_timer = None
        self.title("尋找最佳FVSA")
        self.transient(self.app)
        self.resizable(False, False)
        self.protocol("WM_DELETE_WINDOW", self.destroy)
        body = ttk.Frame(self, padding=18)
        body.pack(fill="both", expand=True)
        axle = "前軸" if panel.key == "front" else "後軸"
        ttk.Label(body, text=f"{axle} · 尋找 Camber 超限前最長的 FVSA",
                  font=("Microsoft JhengHei UI", 11, "bold")).pack(anchor="w")
        start = project["vehicle"]["track_"+panel.key]/2
        ttk.Label(body, text=(f"CSV：{Path(path).name} · {len(lap.travels):,} 筆\n"
                              f"從半輪距 {start:g} m 起，每次加長 {INPUT_STEPS['fvsa']:g} m。\n"
                              f"全圈左右輪最大 |Camber| ≤ {max_camber_deg:g}°，包含直線與過彎。"),
                  wraplength=540).pack(fill="x", pady=(12, 8))
        self.progress = ttk.Progressbar(body, length=540, maximum=100)
        self.progress.pack(fill="x")
        self.status = tk.StringVar(value="正在計算半輪距起點的 Camber…")
        ttk.Label(body, textvariable=self.status, wraplength=540).pack(fill="x", pady=10)
        ttk.Label(body, text="以 CSV 的固定行程與車身 roll／pitch 重算 Camber；不重新模擬圈速，未包含轉向效應。",
                  foreground="#64748b", wraplength=540).pack(fill="x")
        self.close_button = ttk.Button(body, text="取消搜尋", command=self.destroy)
        self.close_button.pack(anchor="e", pady=(12, 0))
        self.update_idletasks()
        x = max(0, self.app.winfo_rootx()+(self.app.winfo_width()-self.winfo_reqwidth())//2)
        y = max(0, self.app.winfo_rooty()+(self.app.winfo_height()-self.winfo_reqheight())//2)
        self.geometry(f"+{x}+{y}")
        self.deiconify()
        self.grab_set()
        self.thread = threading.Thread(target=self.run_search,
                                       args=(project, panel.key, lap, max_camber_deg, self.cancel_event, self.messages),
                                       daemon=True)
        self.thread.start()
        self.poll_timer = self.after(40, self.poll)

    @staticmethod
    def run_search(project, axle, lap, limit, cancel_event, messages):
        try:
            result = search_fvsa(project, axle, lap, limit, cancelled=cancel_event.is_set,
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
                    tested, total, fvsa, peak, reason = value
                    self.progress.configure(value=tested/total*100)
                    text = f"最大 |Camber| {peak.peak_deg:.4f}°" if peak is not None else reason
                    self.status.set(f"已檢查 {tested} 個候選值；FVSA {fvsa:g} m\n{text}")
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
        if result.stop_reason == "camber_limit":
            stop = (f"下一級 {result.stopped_fvsa_m:g} m 的最大 |Camber| "
                    f"{result.stopped_peak.peak_deg:.4f}° 已超過 {self.max_camber_deg:g}°。")
        elif result.stop_reason == "geometry":
            stop = f"在 {result.stopped_fvsa_m:g} m 因幾何／行程無解停止：{result.error}"
        else:
            stop = f"已搜尋至允許上限 {AXLE_LIMITS['fvsa'].high:g} m，尚未超限；採搜尋範圍內最長值。"
        if result.project is None:
            self.status.set(f"半輪距起點即不合格，沒有可套用的 FVSA。\n{stop}\n原設計未變更。")
            return
        if self.app.revision != self.revision:
            self.status.set("搜尋期間專案已變更，結果未套用；請使用目前設計重新搜尋。")
            return
        try:
            self.app.apply_project(result.project)
        except (ValueError, ArithmeticError) as exc:
            self.status.set(f"搜尋結果無法套用：{exc}")
            return
        fvsa = result.project[self.panel.key]["fvsa"]
        peak = result.peak
        self.status.set(f"已套用最佳 FVSA：{fvsa:g} m\n"
                        f"全圈最大 |Camber|：{peak.peak_deg:.4f}°（允許 {self.max_camber_deg:g}°）\n"
                        f"發生於 {peak.wheel}，第 {peak.sample:,} 筆；有號 Camber {peak.signed_deg:+.4f}°\n"
                        f"檢查 {result.tested} 個候選長度，每個合格值均驗證 {peak.total:,} 筆雙輪姿態。\n{stop}")
        self.app.status.set(f"已套用{'前' if self.panel.key == 'front' else '後'}軸最佳 FVSA {fvsa:g} m")

    def destroy(self):
        self.cancel_event.set()
        if self.poll_timer is not None:
            self.after_cancel(self.poll_timer)
            self.poll_timer = None
        super().destroy()
