"""Options for the existing STEP export, before choosing the destination."""
import math
import tkinter as tk
from tkinter import messagebox, ttk


class StepExportDialog(tk.Toplevel):
    def __init__(self, app, options=None):
        super().__init__(app)
        self.withdraw()
        self.options = None
        defaults = options or {}
        self.include_tires = tk.BooleanVar(value=defaults.get("include_tires", True))
        self.include_static_camber = tk.BooleanVar(value=defaults.get("include_static_camber", True))
        self.export_solids = tk.BooleanVar(value=defaults.get("export_solids", False))
        self.default_diameter = defaults.get("tube_diameter_mm", 20.)
        self.tube_diameter_mm = tk.StringVar(value=f"{self.default_diameter:g}")
        self.title("STEP 匯出選項")
        self.transient(app)
        self.resizable(False, False)
        self.protocol("WM_DELETE_WINDOW", self.destroy)
        self.bind("<Escape>", lambda event: self.destroy())
        self.bind("<Return>", lambda event: self.accept())
        body = ttk.Frame(self, padding=18)
        body.pack(fill="both", expand=True)
        ttk.Label(body, text="STEP 懸吊與輪胎", font=("Microsoft JhengHei UI", 11, "bold")).pack(anchor="w")
        ttk.Checkbutton(body, text="匯出實體", variable=self.export_solids,
                        command=self.refresh).pack(anchor="w", pady=(12, 8))
        diameter_row = ttk.Frame(body)
        diameter_row.pack(anchor="w", padx=(20, 0))
        ttk.Label(diameter_row, text="懸吊管徑（mm）").pack(side="left")
        self.diameter_entry = ttk.Entry(diameter_row, width=12, textvariable=self.tube_diameter_mm)
        self.diameter_entry.pack(side="left", padx=(8, 0))
        self.tire_check = ttk.Checkbutton(body, variable=self.include_tires, command=self.refresh)
        self.tire_check.pack(anchor="w", pady=(14, 8))
        self.camber_check = ttk.Checkbutton(body, text="輪胎套用已設定的靜態 Camber",
                                            variable=self.include_static_camber)
        self.camber_check.pack(anchor="w", padx=(20, 0))
        ttk.Label(body, text=(f"靜態 Camber：前軸 {app.project['front']['static_camber_deg']:+g}°，"
                              f"後軸 {app.project['rear']['static_camber_deg']:+g}°\n"
                  "實體：懸吊桿件為指定管徑的實心圓柱，輪胎為封閉圓柱。\n"
                  "未勾實體：懸吊為線框，每顆輪胎用兩個圓圈表示。\n"
                  "輪胎使用自由半徑與胎寬；實體模式不包含 IC／力線參考。\n"
                  "取消靜態 Camber 時，仍保留目前行程造成的車輪姿態。\n"
                  "懸吊中心線與輪心位置沿用目前姿態。"), wraplength=470,
                  foreground="#64748b").pack(fill="x", pady=(12, 14))
        buttons = ttk.Frame(body)
        buttons.pack(fill="x")
        ttk.Button(buttons, text="下一步：選擇儲存位置", command=self.accept).pack(side="right")
        ttk.Button(buttons, text="取消", command=self.destroy).pack(side="right", padx=(0, 8))
        self.refresh()
        self.update_idletasks()
        x = max(0, app.winfo_rootx()+(app.winfo_width()-self.winfo_reqwidth())//2)
        y = max(0, app.winfo_rooty()+(app.winfo_height()-self.winfo_reqheight())//2)
        self.geometry(f"+{x}+{y}")
        self.deiconify()
        self.grab_set()

    def refresh(self):
        self.camber_check.state(["!disabled"] if self.include_tires.get() else ["disabled"])
        solid = self.export_solids.get()
        self.diameter_entry.state(["!disabled"] if solid else ["disabled"])
        self.tire_check.configure(text="包含輪胎（圓柱實體）" if solid else "包含輪胎（每顆輪胎用兩個圓圈表示）")

    def accept(self):
        diameter = self.default_diameter
        if self.export_solids.get():
            try:
                diameter = float(self.tube_diameter_mm.get())
                if not math.isfinite(diameter) or diameter <= 0:
                    raise ValueError
            except ValueError:
                messagebox.showerror("懸吊管徑無效", "請輸入大於 0 的有限數值，單位為 mm。", parent=self)
                self.diameter_entry.focus_set()
                self.diameter_entry.selection_range(0, "end")
                return
        self.options = {"include_tires": self.include_tires.get(),
                        "include_static_camber": self.include_static_camber.get(),
                        "export_solids": self.export_solids.get(),
                        "tube_diameter_mm": diameter}
        self.destroy()

    def destroy(self):
        super().destroy()
        # Destroyed widgets retain Python master links. Release their saved
        # references and Tk variables here, on the UI thread, before a solver
        # worker's garbage collection can finalize a closed dialog cycle.
        self.camber_check = None
        self.include_tires = None
        self.include_static_camber = None
        self.export_solids = None
        self.tube_diameter_mm = None
        self.diameter_entry = None
        self.tire_check = None
