"""Local Tk widgets for the standalone SuspensionGeometryLab application."""
import tkinter as tk
from tkinter import font, ttk


def input_range_label(parent, entry, hint, accepts):
    """Attach the shared range caption and return its draft validation callback."""
    label = ttk.Label(parent, text=hint, foreground="#64748b", wraplength=280)
    label.pack(fill="x", pady=(0, 4))
    label.bind("<Configure>", lambda event: label.configure(wraplength=max(180, event.width - 4)))
    entry.range_label = label

    def refresh(value=None):
        readonly = str(entry.cget("state")) in ("disabled", "readonly")
        try:
            valid = readonly or accepts(float(entry.get() if value is None else value))
        except ValueError:
            valid = False
        entry.configure(style="TEntry" if valid else "Invalid.TEntry")
        label.configure(text=hint + ("；請修正輸入" if not valid else ""),
                        foreground="#64748b" if valid else "#b45309")

    entry.bind("<FocusOut>", lambda _: refresh(), add=True)
    refresh()
    return refresh


class ScrollFrame(ttk.Frame):
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        canvas = tk.Canvas(self, highlightthickness=0, bg="#f1f5f9", width=300)
        scrollbar = ttk.Scrollbar(self, orient="vertical", command=canvas.yview)
        self.content = ttk.Frame(canvas)
        window = canvas.create_window((0, 0), window=self.content, anchor="nw")
        self.content.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda e: canvas.itemconfigure(window, width=e.width))
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        canvas.bind("<MouseWheel>", lambda e: canvas.yview_scroll(-int(e.delta / 120), "units"))
        self.canvas = canvas


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
