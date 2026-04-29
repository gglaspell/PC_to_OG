#!/usr/bin/env python3
"""
PC to Occupancy Grid — Docker Launcher GUI
Tkinter front-end for pointcloud-converter Docker image.
Supports point cloud files: .pcd, .ply, .las, .laz
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
import subprocess
import threading
import os
import platform
from pathlib import Path


# ──────────────────────────────────────────────────────────────────────────────
# Colour palette
# ──────────────────────────────────────────────────────────────────────────────
BG         = "#1e1f22"
SURFACE    = "#2b2d30"
SURFACE2   = "#313438"
BORDER     = "#43454a"
TEXT       = "#dfe1e5"
TEXT_MUTED = "#9da0a6"
ACCENT     = "#4f98a3"
ACCENT_HOV = "#3a7f8b"
SUCCESS    = "#6daa45"
WARN       = "#e8af34"
ERROR      = "#dd6974"
MONO       = "Consolas"


# ──────────────────────────────────────────────────────────────────────────────
# TTK style setup
# ──────────────────────────────────────────────────────────────────────────────
def _apply_style():
    s = ttk.Style()
    s.theme_use("clam")

    base_font = ("Segoe UI", 10) if platform.system() == "Windows" \
           else ("SF Pro Text", 10) if platform.system() == "Darwin" \
           else ("Sans", 10)

    s.configure(".",
        background=BG, foreground=TEXT,
        fieldbackground=SURFACE2, bordercolor=BORDER,
        troughcolor=SURFACE, focuscolor=ACCENT,
        selectbackground=ACCENT, selectforeground=BG,
        font=base_font)

    s.configure("TNotebook",     background=BG, borderwidth=0)
    s.configure("TNotebook.Tab", background=SURFACE, foreground=TEXT_MUTED,
                                 padding=(14, 6), borderwidth=0)
    s.map("TNotebook.Tab",
          background=[("selected", SURFACE2)],
          foreground=[("selected", TEXT)])

    s.configure("TFrame",        background=BG)
    s.configure("TLabel",        background=BG, foreground=TEXT)
    s.configure("Muted.TLabel",  background=BG, foreground=TEXT_MUTED)

    s.configure("TEntry",
        fieldbackground=SURFACE2, foreground=TEXT,
        bordercolor=BORDER, insertcolor=TEXT, padding=(6, 4))
    s.map("TEntry", bordercolor=[("focus", ACCENT)])

    s.configure("TSpinbox",
        fieldbackground=SURFACE2, foreground=TEXT,
        bordercolor=BORDER, arrowcolor=TEXT_MUTED, padding=(6, 4))
    s.map("TSpinbox", bordercolor=[("focus", ACCENT)])

    s.configure("Accent.TButton",
        background=ACCENT, foreground=BG,
        borderwidth=0, padding=(16, 8), font=(*base_font[:1], 10, "bold"))
    s.map("Accent.TButton",
          background=[("active", ACCENT_HOV), ("pressed", ACCENT_HOV),
                      ("disabled", SURFACE2)])

    s.configure("Ghost.TButton",
        background=SURFACE, foreground=TEXT,
        borderwidth=1, relief="flat", padding=(10, 6))
    s.map("Ghost.TButton", background=[("active", SURFACE2)])

    s.configure("Stop.TButton",
        background=ERROR, foreground=BG,
        borderwidth=0, padding=(10, 6), font=(*base_font[:1], 9, "bold"))
    s.map("Stop.TButton", background=[("active", "#c24a59")])

    s.configure("TScrollbar",
        background=SURFACE, troughcolor=BG,
        arrowcolor=TEXT_MUTED, borderwidth=0)
    s.configure("TSeparator", background=BORDER)


# ──────────────────────────────────────────────────────────────────────────────
# Reusable labelled section
# ──────────────────────────────────────────────────────────────────────────────
class Section(ttk.Frame):
    """A titled, separator-topped group of label+widget rows."""

    def __init__(self, parent, title, **kw):
        super().__init__(parent, style="TFrame", **kw)
        ttk.Label(self, text=title.upper(),
                  font=("", 8, "bold"),
                  foreground=ACCENT, background=BG).grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 4))
        ttk.Separator(self, orient="horizontal").grid(
            row=1, column=0, columnspan=3, sticky="ew", pady=(0, 8))
        self._row = 2
        self.columnconfigure(1, weight=1)

    def add_row(self, label, widget, note=""):
        ttk.Label(self, text=label, width=24, anchor="w").grid(
            row=self._row, column=0, sticky="w", padx=(0, 8), pady=3)
        widget.grid(
            row=self._row, column=1, sticky="ew", padx=(0, 8), pady=3)
        if note:
            ttk.Label(self, text=note, style="Muted.TLabel",
                      font=("", 8)).grid(
                row=self._row, column=2, sticky="w", pady=3)
        self._row += 1

    def add_widget(self, widget):
        """Add a full-width widget (no label)."""
        widget.grid(row=self._row, column=0, columnspan=3,
                    sticky="ew", pady=(4, 2))
        self._row += 1


# ──────────────────────────────────────────────────────────────────────────────
# Main application
# ──────────────────────────────────────────────────────────────────────────────
class App(tk.Tk):

    # ── defaults (mirror argparse) ────────────────────────────────────────────
    DEFAULTS = {
        "octree_res":     "0.1",
        "grid_res":       "0.05",
        "slope_deg":      "15.0",
        "z_min":          "0.1",
        "z_max":          "2.0",
        "normal_radius":  "0.2",
        "downsample":     "0.05",
        "workers":        "4",
        "min_cluster_size": "30",
        "cluster_eps":    "0.2",
    }

    def __init__(self):
        super().__init__()
        self.title("PC → Occupancy Grid  |  Docker Launcher")
        self.geometry("900x820")
        self.minsize(780, 640)
        self.configure(bg=BG)
        _apply_style()

        self._proc    = None
        self._running = False

        # ── string variables ──────────────────────────────────────────────────
        self.v_input       = tk.StringVar()
        self.v_output_dir  = tk.StringVar()
        self.v_output_name = tk.StringVar(value="my_map")
        self.v_image       = tk.StringVar(value="pointcloud-converter:latest")

        self.v_params = {k: tk.StringVar(value=v)
                         for k, v in self.DEFAULTS.items()}

        # ── build UI ──────────────────────────────────────────────────────────
        self._build_header()
        self._nb = ttk.Notebook(self)
        self._nb.pack(fill="both", expand=True, padx=12, pady=(0, 8))
        self._build_config_tab(self._nb)
        self._build_log_tab(self._nb)
        self._build_footer()

        # live command preview
        for v in [self.v_input, self.v_output_dir,
                  self.v_output_name, self.v_image,
                  *self.v_params.values()]:
            v.trace_add("write", self._update_preview)

        self._update_preview()

    # ── header ────────────────────────────────────────────────────────────────
    def _build_header(self):
        hdr = tk.Frame(self, bg=SURFACE, height=56)
        hdr.pack(fill="x")
        hdr.pack_propagate(False)

        cnv = tk.Canvas(hdr, width=36, height=36, bg=SURFACE,
                        highlightthickness=0)
        cnv.pack(side="left", padx=(16, 8), pady=10)
        cnv.create_rectangle(2,  2,  34, 34, outline=ACCENT, width=2, fill="")
        cnv.create_rectangle(8,  8,  17, 17, fill=ACCENT,  outline="")
        cnv.create_rectangle(19, 8,  28, 17, fill=BORDER,  outline="")
        cnv.create_rectangle(8,  19, 17, 28, fill=BORDER,  outline="")
        cnv.create_rectangle(19, 19, 28, 28, fill=ACCENT,  outline="")

        tk.Label(hdr, text="PC → Occupancy Grid",
                 font=("", 13, "bold"), bg=SURFACE, fg=TEXT).pack(side="left")
        tk.Label(hdr, text="Docker Launcher",
                 font=("", 10), bg=SURFACE, fg=TEXT_MUTED).pack(
            side="left", padx=(8, 0))

    # ── config tab ────────────────────────────────────────────────────────────
    def _build_config_tab(self, nb):
        outer = ttk.Frame(nb)
        nb.add(outer, text="  ⚙  Configuration  ")

        # scrollable canvas
        canvas = tk.Canvas(outer, bg=BG, highlightthickness=0)
        vsb    = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        inner = ttk.Frame(canvas)
        win   = canvas.create_window((0, 0), window=inner, anchor="nw")

        inner.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>",
                    lambda e: canvas.itemconfig(win, width=e.width))
        canvas.bind_all("<MouseWheel>",
                        lambda e: canvas.yview_scroll(
                            int(-1 * (e.delta / 120)), "units"))

        inner.columnconfigure(0, weight=1)
        PX = dict(padx=16, sticky="ew")   # shared grid kwargs (no pady)

        # ── Section 1: I/O ────────────────────────────────────────────────────
        io = Section(inner, "Input / Output")
        io.grid(row=0, column=0, pady=(14, 6), **PX)

        # input file picker
        inp_row = ttk.Frame(io)
        ttk.Entry(inp_row, textvariable=self.v_input, width=40).pack(
            side="left", fill="x", expand=True)
        ttk.Button(inp_row, text="Browse…", style="Ghost.TButton",
                   command=self._browse_input).pack(side="left", padx=(6, 0))
        io.add_row("Input file", inp_row, ".pcd  .ply  .las  .laz")

        # output dir picker
        out_row = ttk.Frame(io)
        ttk.Entry(out_row, textvariable=self.v_output_dir, width=40).pack(
            side="left", fill="x", expand=True)
        ttk.Button(out_row, text="Browse…", style="Ghost.TButton",
                   command=self._browse_output).pack(side="left", padx=(6, 0))
        io.add_row("Output directory", out_row)

        io.add_row("Output filename",
                   ttk.Entry(io, textvariable=self.v_output_name),
                   "no extension  →  saves .pgm + .yaml")
        io.add_row("Docker image",
                   ttk.Entry(io, textvariable=self.v_image))

        # ── Section 2: OcTree & Grid ──────────────────────────────────────────
        og = Section(inner, "OcTree & Grid Resolution")
        og.grid(row=1, column=0, pady=6, **PX)

        og.add_row("Octree resolution",
                   self._spinbox(og, "octree_res", 0.01, 2.0,  0.01, "%.3f"),
                   "metres — internal 3D octree")
        og.add_row("Grid resolution",
                   self._spinbox(og, "grid_res",   0.01, 1.0,  0.01, "%.3f"),
                   "metres / cell — final 2D map")

        # ── Section 3: Ground Separation ──────────────────────────────────────
        gs = Section(inner, "Ground Separation")
        gs.grid(row=2, column=0, pady=6, **PX)

        gs.add_row("Slope threshold",
                   self._spinbox(gs, "slope_deg",    0.0,  90.0, 0.5,  "%.1f"),
                   "degrees — max angle to classify as ground")
        gs.add_row("Normal est. radius",
                   self._spinbox(gs, "normal_radius", 0.01,  2.0, 0.01, "%.2f"),
                   "metres — radius for surface normal estimation")
        gs.add_row("Downsample voxel",
                   self._spinbox(gs, "downsample",    0.0,   1.0, 0.01, "%.3f"),
                   "metres — voxel size before normal est.  (0 = off)")

        # ── Section 4: Obstacle Height ────────────────────────────────────────
        oh = Section(inner, "Obstacle Height Band  (relative to ground)")
        oh.grid(row=3, column=0, pady=6, **PX)

        oh.add_row("Z min offset",
                   self._spinbox(oh, "z_min", 0.0,  5.0, 0.05, "%.2f"),
                   "metres — min Z above ground to check")
        oh.add_row("Z max offset",
                   self._spinbox(oh, "z_max", 0.1, 20.0, 0.1,  "%.2f"),
                   "metres — max Z above ground to check")

        # ── Section 5: Noise Filter ───────────────────────────────────────────
        nf = Section(inner, "Noise / Small Cluster Filter")
        nf.grid(row=4, column=0, pady=6, **PX)

        nf.add_row("Min cluster size",
                   self._spinbox(nf, "min_cluster_size", 0, 500, 5, "%d"),
                   "points — clusters smaller than this are removed  (0 = off)")
        nf.add_row("Cluster ε (eps)",
                   self._spinbox(nf, "cluster_eps", 0.01, 2.0, 0.01, "%.2f"),
                   "metres — DBSCAN neighbourhood radius")

        # ── Section 6: Performance ────────────────────────────────────────────
        pf = Section(inner, "Performance")
        pf.grid(row=5, column=0, pady=(6, 18), **PX)

        pf.add_row("Worker threads",
                   self._spinbox(pf, "workers", 1, 64, 1, "%d"),
                   "parallel threads for grid generation")

        # ── reset-to-defaults button ──────────────────────────────────────────
        reset_frm = ttk.Frame(inner)
        reset_frm.grid(row=6, column=0, pady=(0, 16), **PX)
        ttk.Button(reset_frm, text="↺  Reset all parameters to defaults",
                   style="Ghost.TButton",
                   command=self._reset_defaults).pack(side="right")

    # ── log tab ───────────────────────────────────────────────────────────────
    def _build_log_tab(self, nb):
        frm = ttk.Frame(nb)
        nb.add(frm, text="  📋  Output Log  ")

        tb = ttk.Frame(frm)
        tb.pack(fill="x", padx=8, pady=(8, 4))
        ttk.Button(tb, text="Clear log", style="Ghost.TButton",
                   command=self._clear_log).pack(side="left")
        self._stop_btn = ttk.Button(
            tb, text="⏹  Stop", style="Stop.TButton",
            command=self._stop_process, state="disabled")
        self._stop_btn.pack(side="right")

        self._log = scrolledtext.ScrolledText(
            frm, wrap="word", state="disabled",
            bg="#0d0e0f", fg=TEXT, insertbackground=TEXT,
            font=(MONO, 9), relief="flat",
            selectbackground=ACCENT, selectforeground=BG,
            borderwidth=0, padx=10, pady=8)
        self._log.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        self._log.tag_config("info",    foreground=TEXT)
        self._log.tag_config("success", foreground=SUCCESS)
        self._log.tag_config("warn",    foreground=WARN)
        self._log.tag_config("error",   foreground=ERROR)
        self._log.tag_config("accent",  foreground=ACCENT)
        self._log.tag_config("muted",   foreground=TEXT_MUTED)

    # ── footer: command preview + run ─────────────────────────────────────────
    def _build_footer(self):
        ttk.Separator(self, orient="horizontal").pack(fill="x", padx=12)

        foot = ttk.Frame(self)
        foot.pack(fill="x", padx=12, pady=8)
        foot.columnconfigure(0, weight=1)

        lbl_row = ttk.Frame(foot)
        lbl_row.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 4))
        ttk.Label(lbl_row, text="Docker command preview",
                  font=("", 8, "bold"), foreground=ACCENT).pack(side="left")
        ttk.Button(lbl_row, text="Copy", style="Ghost.TButton",
                   command=self._copy_cmd).pack(side="right")

        self._cmd_box = tk.Text(
            foot, height=4, wrap="word",
            bg=SURFACE2, fg=TEXT_MUTED,
            font=(MONO, 8), relief="flat",
            insertbackground=TEXT, state="disabled",
            borderwidth=0, padx=8, pady=6)
        self._cmd_box.grid(row=1, column=0, columnspan=2,
                           sticky="ew", pady=(0, 8))

        self._run_btn = ttk.Button(
            foot, text="▶  Run Conversion",
            style="Accent.TButton", command=self._run)
        self._run_btn.grid(row=2, column=0, columnspan=2,
                           sticky="ew", pady=(0, 4))

    # ── spinbox factory ───────────────────────────────────────────────────────
    def _spinbox(self, parent, key, lo, hi, inc, fmt):
        return ttk.Spinbox(parent,
                           textvariable=self.v_params[key],
                           from_=lo, to=hi, increment=inc,
                           width=10, format=fmt)

    # ── browse helpers ────────────────────────────────────────────────────────
    def _browse_input(self):
        p = filedialog.askopenfilename(
            title="Select point cloud file",
            filetypes=[("Point clouds", "*.ply *.pcd *.las *.laz"),
                       ("All files", "*.*")])
        if p:
            self.v_input.set(p)

    def _browse_output(self):
        p = filedialog.askdirectory(title="Select output directory")
        if p:
            self.v_output_dir.set(p)

    # ── reset defaults ────────────────────────────────────────────────────────
    def _reset_defaults(self):
        for key, val in self.DEFAULTS.items():
            self.v_params[key].set(val)

    # ── docker command builder ────────────────────────────────────────────────
    def _build_parts(self):
        inp      = self.v_input.get().strip()
        out_dir  = self.v_output_dir.get().strip()
        out_nm   = self.v_output_name.get().strip() or "my_map"
        image    = self.v_image.get().strip() or "pointcloud-converter:latest"

        inp_path = Path(inp) if inp else None
        if inp_path and inp_path.is_file():
            data_host = str(inp_path.parent)
            cont_inp  = f"/data/{inp_path.name}"
        else:
            data_host = str(inp_path.parent) if inp_path else "<directory_of_input_file>"
            cont_inp  = f"/data/{inp_path.name}" if inp_path else "/data/<input_file>"

        out_host = out_dir if out_dir else "<output_directory>"
        cont_out = f"/output/{out_nm}"

        parts = [
            "docker", "run", "--rm",
            "-v", f"{data_host}:/data",
            "-v", f"{out_host}:/output",
            image,
            cont_inp, cont_out,
        ]

        # append only non-default params
        for key, default in self.DEFAULTS.items():
            val = self.v_params[key].get().strip()
            try:
                if float(val) != float(default):
                    parts.extend([f"--{key}", val])
            except ValueError:
                pass

        return parts

    def _parts_to_str(self, parts):
        """Pretty multi-line docker run string."""
        lines = []
        i = 2  # skip 'docker run'
        while i < len(parts):
            p = parts[i]
            if p == "-v" and i + 1 < len(parts):
                lines.append(f'  -v "{parts[i+1]}"')
                i += 2
            elif p.startswith("--") and i + 1 < len(parts) and not parts[i+1].startswith("--"):
                lines.append(f"  {p} {parts[i+1]}")
                i += 2
            else:
                lines.append(f"  {p}")
                i += 1
        return "docker run \\\n" + " \\\n".join(lines)

    def _update_preview(self, *_):
        try:
            text = self._parts_to_str(self._build_parts())
        except Exception:
            text = "# fill in the input file and output directory above"
        self._cmd_box.configure(state="normal")
        self._cmd_box.delete("1.0", "end")
        self._cmd_box.insert("1.0", text)
        self._cmd_box.configure(state="disabled")

    def _copy_cmd(self):
        self.clipboard_clear()
        self.clipboard_append(self._cmd_box.get("1.0", "end").strip())
        self._log_line("Command copied to clipboard.", "accent")

    # ── logging ───────────────────────────────────────────────────────────────
    def _log_line(self, text, tag="info"):
        self._log.configure(state="normal")
        self._log.insert("end", text + "\n", tag)
        self._log.see("end")
        self._log.configure(state="disabled")

    def _clear_log(self):
        self._log.configure(state="normal")
        self._log.delete("1.0", "end")
        self._log.configure(state="disabled")

    # ── run / stop ────────────────────────────────────────────────────────────
    def _validate(self):
        inp = self.v_input.get().strip()
        if not inp:
            messagebox.showerror("Missing input",
                "Please select a point cloud file (.pcd, .ply, .las, .laz).")
            return False
        if not Path(inp).is_file():
            messagebox.showerror("File not found",
                f"Input file not found:\n{inp}")
            return False
        if Path(inp).suffix.lower() not in (".pcd", ".ply", ".las", ".laz"):
            messagebox.showwarning("Unsupported format",
                "Input file should be .pcd, .ply, .las, or .laz.")
        if not self.v_output_dir.get().strip():
            messagebox.showerror("Missing output directory",
                "Please select an output directory.")
            return False
        return True

    def _run(self):
        if self._running:
            return
        if not self._validate():
            return

        parts    = self._build_parts()
        cmd_text = self._parts_to_str(parts)

        self._clear_log()
        self._log_line("╔══════════════════════════════════════════════════╗", "accent")
        self._log_line("║      Point Cloud → Occupancy Grid Conversion     ║", "accent")
        self._log_line("╚══════════════════════════════════════════════════╝\n", "accent")
        self._log_line("Command:\n" + cmd_text + "\n", "muted")
        self._log_line("─" * 52, "muted")

        self._running = True
        self._run_btn.configure(state="disabled", text="⏳  Running…")
        self._stop_btn.configure(state="normal")
        self._nb.select(1)

        threading.Thread(target=self._worker, args=(parts,), daemon=True).start()

    def _worker(self, parts):
        try:
            self._proc = subprocess.Popen(
                parts,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True,
            )

            for line in self._proc.stdout:
                line = line.rstrip()
                tag  = "info"
                if any(k in line for k in ("✓", "complete", "Saved", "✔")):
                    tag = "success"
                elif any(k in line for k in ("❌", "Error", "error", "FAIL")):
                    tag = "error"
                elif any(k in line for k in ("⚠", "Warning", "warn")):
                    tag = "warn"
                elif any(k in line for k in ("🔍", "📊", "🌳", "🔲", "📐",
                                              "💾", "🧹", "Progress")):
                    tag = "accent"
                self.after(0, self._log_line, line, tag)

            self._proc.wait()
            rc = self._proc.returncode
            self.after(0, self._log_line, "─" * 52, "muted")
            if rc == 0:
                self.after(0, self._log_line,
                           "\n✅  Conversion finished successfully!", "success")
            elif rc == -1:
                self.after(0, self._log_line,
                           "\n⏹  Process stopped by user.", "warn")
            else:
                self.after(0, self._log_line,
                           f"\n❌  Process exited with code {rc}.", "error")

        except FileNotFoundError:
            self.after(0, self._log_line,
                "❌  'docker' not found. Is Docker installed and on PATH?",
                "error")
        except Exception as e:
            self.after(0, self._log_line,
                f"❌  Unexpected error: {e}", "error")
        finally:
            self._proc    = None
            self._running = False
            self.after(0, self._run_btn.configure,
                       {"state": "normal", "text": "▶  Run Conversion"})
            self.after(0, self._stop_btn.configure, {"state": "disabled"})

    def _stop_process(self):
        if self._proc:
            try:
                self._proc.terminate()
            except Exception:
                pass
            self._log_line("⏹  Sending stop signal…", "warn")


if __name__ == "__main__":
    app = App()
    app.mainloop()
