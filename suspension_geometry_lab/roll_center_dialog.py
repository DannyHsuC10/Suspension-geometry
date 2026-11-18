"""Cancellable RC search progress; all Tk work stays on the main thread."""
import queue
import threading
import tkinter as tk
from tkinter import ttk

from .input_limits import INPUT_STEPS
from .roll_center_search import SearchCancelled, search_roll_center, search_balanced_rear_roll_center


class RollCenterSearchDialog(tk.Toplevel):
    def __init__(self, panel, project, settings):
        super().__init__(panel.app)
        self.withdraw()
        self.panel, self.app = panel, panel.app
        self.revision = self.app.revision
        self.settings = settings
        self.cancel_event = threading.Event()
        self.messages = queue.Queue()
        self.poll_timer = None
        self.title("尋找最佳roll center")
        self.transient(self.app)
        self.resizable(False, False)
        self.protocol("WM_DELETE_WINDOW", self.destroy)
        body = ttk.Frame(self, padding=18)
        body.pack(fill="both", expand=True)
        axle = "前軸" if panel.key == "front" else "後軸"
        goal = "解析求解前後載荷轉移相等的 RC" if panel.key == "rear" else "尋找最低且全行程保持地面上方的 RC"
        ttk.Label(body, text=f"{axle} · {goal}", wraplength=520,
                  font=("Microsoft JhengHei UI", 11, "bold")).pack(anchor="w")
        step_mm = INPUT_STEPS["rc_height"]*1000
        height_hint = (f"允許設計高度 {settings.height_min_mm/1000:g}～{settings.height_max_mm/1000:g} m，取整級距 {step_mm:g} mm\n"
                       if panel.key == "rear" else f"搜尋起點 {settings.height_min_mm/1000:g} m，級距 {step_mm:g} mm\n")
        ttk.Label(body, text=(height_hint+
                              f"每輪行程 −{settings.droop_mm:g}～+{settings.bump_mm:g} mm；"
                              f"左右輪獨立組合，每輪取樣間隔 ≤ {step_mm:g} mm。"),
                  wraplength=520).pack(fill="x", pady=(12, 8))
        self.progress = ttk.Progressbar(body, length=520, maximum=100)
        self.progress.pack(fill="x")
        self.status = tk.StringVar(value="正在計算解析解…" if panel.key == "rear" else "正在建立候選幾何…")
        ttk.Label(body, textvariable=self.status, wraplength=520).pack(fill="x", pady=10)
        ttk.Label(body, text="有限取樣檢查，適用本工具的 FV 平面近似；亦排除取樣間的奇異位置符號變化。",
                  foreground="#64748b", wraplength=520).pack(fill="x")
        if panel.key == "rear":
            ttk.Label(body, text="固定前軸；直接求解 ΔFz,f＝ΔFz,r，再驗證取整後的單一設計。超界或不合格時不套用。\n"
                      "簡化為全質量簧上、剛性車身，忽略空力、簧下質量、輪胎負載敏感性與動態側傾平衡。\n"
                      "重量比用於側向力分配；前後載荷轉移各占 50% 不代表中性轉向。",
                      foreground="#64748b", wraplength=520).pack(fill="x", pady=(8, 0))
        self.close_button = ttk.Button(body, text="取消搜尋", command=self.destroy)
        self.close_button.pack(anchor="e", pady=(12, 0))
        self.update_idletasks()
        x = max(0, self.app.winfo_rootx()+(self.app.winfo_width()-self.winfo_reqwidth())//2)
        y = max(0, self.app.winfo_rooty()+(self.app.winfo_height()-self.winfo_reqheight())//2)
        self.geometry(f"+{x}+{y}")
        self.deiconify()
        self.grab_set()

        # Pass plain snapshots, a queue and an Event to the worker, never widgets.
        self.thread = threading.Thread(target=self.run_search,
                                       args=(project, panel.key, settings, self.cancel_event, self.messages),
                                       daemon=True)
        self.thread.start()
        self.poll_timer = self.after(40, self.poll)

    @staticmethod
    def run_search(project, axle, settings, cancel_event, messages):
        try:
            options = {"cancelled": cancel_event.is_set,
                       "progress": lambda tested, total, height, reason:
                           messages.put(("progress", (tested, total, height, reason)))}
            result = (search_balanced_rear_roll_center(project, settings, **options) if axle == "rear"
                      else search_roll_center(project, axle, settings, **options))
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
                    tested, total, height, reason = value
                    self.progress.configure(value=tested/total*100)
                    self.status.set(f"已完成 {tested} / {total} 個搜尋步驟；目前 {height/1000:.3f} m\n{reason}")
                elif kind == "error":
                    self.status.set(f"{'解析求解' if self.panel.key == 'rear' else '搜尋'}失敗：{value}")
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
        analytic_text = ""
        if result.analytic is not None:
            analytic_text = (f"解析後軸 RC 高度（接地座標）：{result.analytic.rear_rc_m:.9g} m\n"
                             f"換算的 RC 設計高度：{result.analytic.rear_design_m:.9g} m\n"
                             f"依設定級距取整：{result.rounded_height_m:g} m\n")
        if result.project is None:
            if result.analytic is not None:
                self.status.set(analytic_text+f"未套用：{result.failure_reason}\n原設計未變更。")
                return
            reasons = "\n".join(f"{reason}：{count} 個" for reason, count in
                                sorted(result.rejected.items(), key=lambda pair: -pair[1])[:3])
            self.status.set(f"範圍內找不到合適的 RC 高度，已檢查 {result.tested} 個候選值。\n"
                            f"{reasons}\n原設計未變更。")
            return
        if self.app.revision != self.revision:
            self.status.set("搜尋期間專案已變更，結果未套用；請使用目前設計重新搜尋。")
            return
        try:
            self.app.apply_project(result.project)
        except (ValueError, ArithmeticError) as exc:
            self.status.set(f"搜尋結果無法套用：{exc}")
            return
        height = result.project[self.panel.key]["rc_height"]
        left, right = result.check.worst_travel_mm
        grid_label = "前後軸各" if result.analytic is not None else "該軸"
        balance_text = ""
        if result.balance is not None:
            balance = result.balance
            balance_text = (analytic_text+"目標：前後載荷轉移量相等（前軸占 50%）\n"
                            f"套用後前軸載荷轉移占比：{balance.front_transfer_fraction*100:.4f}%\n"
                            f"取整後平衡誤差：{balance.error*100:+.4f} 個百分點\n")
            if result.reference_balance is not None:
                previous = result.reference_balance
                balance_text += (f"求解前載荷轉移占比：{previous.front_transfer_fraction*100:.4f}%"
                                 f"（誤差 {previous.error*100:+.4f} 個百分點）\n")
            balance_text += (f"零行程實際前／後 RC：{balance.front_rc_m*1000:.3f}／{balance.rear_rc_m*1000:.3f} mm\n"
                             f"固定前軸全行程最低 RC：{result.front_check.min_height_mm:.3f} mm\n")
        self.status.set(f"已套用最佳 RC 設計高度：{height:.3f} m（{height*1000:g} mm）\n"+balance_text+
                        f"工作行程內最低 RC：{result.check.min_height_mm:.3f} mm\n"
                        f"最低點左右行程：{left:+g} / {right:+g} mm\n"
                        f"最大橫向 |x′|：{result.check.max_abs_x_mm:.3f} mm\n"
                        f"驗證 {result.tested} 個設計高度；{grid_label} {result.travel_samples**2:,} 組姿態。")
        self.app.status.set(f"已套用{'前' if self.panel.key == 'front' else '後'}軸最佳 RC 高度 {height:.3f} m")

    def destroy(self):
        self.cancel_event.set()
        if self.poll_timer is not None:
            self.after_cancel(self.poll_timer)
            self.poll_timer = None
        super().destroy()
