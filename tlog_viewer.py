"""ArduPilot tlog viewer/decoder - Tkinter GUI.

Loads a MAVLink telemetry log (.tlog), lets you browse and search decoded
messages, and plot numeric fields over time (including comparing fields
from different message types on the same chart).
"""
import os
import threading
from datetime import datetime

import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk, filedialog, messagebox

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.dates as mdates
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

from pymavlink import mavutil

NUMERIC_TYPES = (int, float)
SKIP_FIELDS = {"mavpackettype"}
MAX_TABLE_ROWS = 20000
MAV_CMD_ENUM = mavutil.mavlink.enums.get("MAV_CMD", {})

# Column auto-sizing: measure header + every currently-shown cell's text
# and size each column to fit exactly (floored at MIN_COLUMN_WIDTH so an
# all-empty column stays clickable) — no upper cap, so nothing is ever
# truncated; a column wider than the window just means more to reach via
# the horizontal scrollbar / Shift+wheel (columns are pinned to
# stretch=False specifically so that scrolling has real range to work
# with instead of Treeview re-stretching everything to fit the frame).
COLUMN_HEADER_PAD = 16
COLUMN_CELL_PAD = 14
MIN_COLUMN_WIDTH = 40

# Prefix marking a synthetic "filter by this decoded MAV_CMD" entry folded
# into the Type dropdown, alongside the real message types.
COMMAND_FILTER_PREFIX = "Command: "


def _humanize_mav_cmd(enum_name):
    """"MAV_CMD_NAV_TAKEOFF" -> "Nav Takeoff" — drop the redundant prefix
    every MAV_CMD shares and title-case the rest so it reads as a label
    instead of a wire-format constant name."""
    if enum_name.startswith("MAV_CMD_"):
        enum_name = enum_name[len("MAV_CMD_"):]
    return enum_name.replace("_", " ").title()


class TlogData:
    """Holds messages decoded from a single tlog file.

    HEARTBEAT entries get a synthetic "mode" field (human-readable flight
    mode name, e.g. "STABILIZE") added to `fields` via
    `mavutil.mode_string_v10`, so it shows up as a normal column/search
    target alongside the raw `custom_mode` integer. Any entry with a
    "command" field (COMMAND_LONG, COMMAND_INT, COMMAND_ACK, MISSION_ITEM,
    ...) gets a synthetic "command_name" field looked up from the MAV_CMD
    enum and humanized via `_humanize_mav_cmd` (e.g. "MAV_CMD_NAV_TAKEOFF"
    -> "Nav Takeoff"), alongside the raw numeric id.
    """

    def __init__(self):
        self.messages = []        # list of {"time", "type", "fields", "sysid", "compid"}
        self.by_type = {}         # type -> list of entries (same dicts as above)
        self.numeric_fields = {}  # type -> sorted list of numeric field names
        self.sysids = []          # sorted list of distinct source system ids seen
        self.guessed_outgoing_sysid = None
        self.start_time = None
        self.end_time = None

    def load(self, path, progress_cb=None):
        conn = mavutil.mavlink_connection(path, dialect="ardupilotmega", robust_parsing=True)
        count = 0
        sysids_seen = set()
        while True:
            msg = conn.recv_match(blocking=False)
            if msg is None:
                break
            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                continue
            ts = getattr(msg, "_timestamp", None)
            if ts is None:
                continue

            fields = {k: v for k, v in msg.to_dict().items() if k not in SKIP_FIELDS}
            if msg_type == "HEARTBEAT":
                try:
                    fields["mode"] = mavutil.mode_string_v10(msg)
                except Exception:
                    fields["mode"] = str(fields.get("custom_mode", ""))
            cmd_id = fields.get("command")
            if isinstance(cmd_id, int):
                cmd_enum = MAV_CMD_ENUM.get(cmd_id)
                fields["command_name"] = (
                    _humanize_mav_cmd(cmd_enum.name) if cmd_enum else f"Unknown ({cmd_id})"
                )
            sysid = msg.get_srcSystem()
            entry = {
                "time": ts, "type": msg_type, "fields": fields,
                "sysid": sysid, "compid": msg.get_srcComponent(),
            }
            self.messages.append(entry)
            self.by_type.setdefault(msg_type, []).append(entry)
            sysids_seen.add(sysid)

            count += 1
            if progress_cb and count % 2000 == 0:
                progress_cb(count)

        self.messages.sort(key=lambda e: e["time"])
        if self.messages:
            self.start_time = self.messages[0]["time"]
            self.end_time = self.messages[-1]["time"]

        for msg_type, entries in self.by_type.items():
            sample = entries[0]["fields"]
            self.numeric_fields[msg_type] = sorted(
                k for k, v in sample.items()
                if isinstance(v, NUMERIC_TYPES) and not isinstance(v, bool)
            )

        self.sysids = sorted(sysids_seen)
        self.guessed_outgoing_sysid = self._guess_outgoing_sysid()

        return count

    def _guess_outgoing_sysid(self):
        """Best-effort guess of which sysid is "us" (the GCS), so incoming
        (vehicle) vs outgoing (GCS) messages can be colored differently.
        Prefers a HEARTBEAT explicitly typed MAV_TYPE_GCS; falls back to the
        higher of exactly two sysids (ArduPilot vehicles default to sysid 1,
        GCS software conventionally defaults to 255)."""
        for e in self.by_type.get("HEARTBEAT", []):
            if e["fields"].get("type") == mavutil.mavlink.MAV_TYPE_GCS:
                return e["sysid"]
        if len(self.sysids) == 2:
            return max(self.sysids)
        return None


OUTGOING_BG = "#d8e8ff"
INCOMING_BG = "#ffe3cf"


class MessagesTab(ttk.Frame):
    """Filterable table of decoded messages, colored by direction (incoming
    from the vehicle vs outgoing from the GCS, inferred from sysid; no
    separate text column since the color already encodes it), with
    row/cell copy support. Direction can also be filtered explicitly
    (All/Outgoing/Incoming) via a dropdown. Clicking a column header sorts
    the table by that column (click again to reverse); HEARTBEAT rows get
    a synthetic "mode" field showing the flight mode as plain text (e.g.
    "STABILIZE") instead of just the raw custom_mode integer, and any row
    with a "command" field (COMMAND_LONG, COMMAND_INT, COMMAND_ACK, ...)
    gets a synthetic "command_name" field as a readable label (e.g.
    "Nav Takeoff"). Every distinct command_name is folded into the same
    **Type** dropdown as a "Command: <name>" entry, so filtering to one
    specific command is just another Type selection rather than a second
    dropdown. Columns auto-size to fit their content (header + a sample of
    cell text, see `_compute_column_widths`) and don't stretch to fill the
    frame, so the table scrolls horizontally (drag the scrollbar or
    Shift+wheel) for anything still too wide to fit."""

    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self._context_click_pos = None
        self.sort_column = None
        self.sort_reverse = False
        self._build()

    def _build(self):
        controls = ttk.Frame(self, padding=5)
        controls.pack(side="top", fill="x")

        ttk.Label(controls, text="Type:").pack(side="left")
        self.type_var = tk.StringVar(value="All")
        self.type_combo = ttk.Combobox(controls, textvariable=self.type_var, state="readonly", width=32)
        self.type_combo.pack(side="left", padx=5)
        self.type_combo.bind("<<ComboboxSelected>>", lambda e: self.apply_filter())

        ttk.Label(controls, text="Search:").pack(side="left", padx=(15, 0))
        self.search_var = tk.StringVar()
        search_entry = ttk.Entry(controls, textvariable=self.search_var, width=30)
        search_entry.pack(side="left", padx=5)
        search_entry.bind("<Return>", lambda e: self.apply_filter())
        ttk.Button(controls, text="Filter", command=self.apply_filter).pack(side="left")
        ttk.Button(controls, text="Clear", command=self.clear_filter).pack(side="left", padx=5)

        ttk.Label(controls, text="Direction:").pack(side="left", padx=(15, 0))
        self.direction_var = tk.StringVar(value="All")
        self.direction_combo = ttk.Combobox(
            controls, textvariable=self.direction_var, state="readonly",
            width=10, values=["All", "Outgoing", "Incoming"],
        )
        self.direction_combo.pack(side="left", padx=5)
        self.direction_combo.bind("<<ComboboxSelected>>", lambda e: self.apply_filter())

        ttk.Label(controls, text="Outgoing sysid:").pack(side="left", padx=(15, 0))
        self.outgoing_sysid_var = tk.StringVar()
        self.outgoing_combo = ttk.Combobox(
            controls, textvariable=self.outgoing_sysid_var, state="readonly", width=6
        )
        self.outgoing_combo.pack(side="left", padx=5)
        self.outgoing_combo.bind("<<ComboboxSelected>>", lambda e: self.apply_filter())

        legend = ttk.Frame(controls)
        legend.pack(side="left", padx=(15, 0))
        tk.Label(legend, text="  ", bg=OUTGOING_BG, relief="solid", borderwidth=1).pack(side="left")
        ttk.Label(legend, text="outgoing").pack(side="left", padx=(2, 8))
        tk.Label(legend, text="  ", bg=INCOMING_BG, relief="solid", borderwidth=1).pack(side="left")
        ttk.Label(legend, text="incoming").pack(side="left", padx=(2, 0))

        self.count_label = ttk.Label(controls, text="")
        self.count_label.pack(side="right")

        tree_frame = ttk.Frame(self)
        tree_frame.pack(fill="both", expand=True)

        self.tree = ttk.Treeview(tree_frame, show="headings")
        self.tree.tag_configure("outgoing", background=OUTGOING_BG)
        self.tree.tag_configure("incoming", background=INCOMING_BG)
        vsb = ttk.Scrollbar(tree_frame, orient="vertical", command=self.tree.yview)
        hsb = ttk.Scrollbar(tree_frame, orient="horizontal", command=self.tree.xview)
        self.tree.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)
        self.tree.grid(row=0, column=0, sticky="nsew")
        vsb.grid(row=0, column=1, sticky="ns")
        hsb.grid(row=1, column=0, sticky="ew")
        tree_frame.rowconfigure(0, weight=1)
        tree_frame.columnconfigure(0, weight=1)

        self.tree.bind("<Control-c>", self.copy_selected_rows)
        self.tree.bind("<Button-3>", self._show_context_menu)
        # Horizontal scroll for long rows (in addition to dragging the
        # scrollbar): Shift+wheel on Windows/macOS, Shift+Button-4/5 on X11.
        self.tree.bind("<Shift-MouseWheel>", self._on_shift_mousewheel)
        self.tree.bind("<Shift-Button-4>", lambda e: self.tree.xview_scroll(-2, "units"))
        self.tree.bind("<Shift-Button-5>", lambda e: self.tree.xview_scroll(2, "units"))
        self.context_menu = tk.Menu(self.tree, tearoff=0)
        self.context_menu.add_command(label="Copy row(s)", command=self.copy_selected_rows)
        self.context_menu.add_command(label="Copy cell", command=self.copy_clicked_cell)

    def _on_shift_mousewheel(self, event):
        step = -2 if event.delta > 0 else 2
        self.tree.xview_scroll(step, "units")
        return "break"

    def on_data_loaded(self):
        data = self.app.data
        command_names = sorted({
            e["fields"]["command_name"] for e in data.messages if "command_name" in e["fields"]
        })
        types = (
            ["All"] + sorted(data.by_type.keys())
            + [f"{COMMAND_FILTER_PREFIX}{name}" for name in command_names]
        )
        self.type_combo["values"] = types
        self.type_var.set("All")

        self.outgoing_combo["values"] = [str(s) for s in data.sysids]
        if data.guessed_outgoing_sysid is not None:
            self.outgoing_sysid_var.set(str(data.guessed_outgoing_sysid))
        elif data.sysids:
            self.outgoing_sysid_var.set(str(data.sysids[0]))
        else:
            self.outgoing_sysid_var.set("")

        self.apply_filter()

    def clear_filter(self):
        self.search_var.set("")
        self.type_var.set("All")
        self.direction_var.set("All")
        self.apply_filter()

    def apply_filter(self):
        data = self.app.data
        msg_type = self.type_var.get()
        search = self.search_var.get().strip().lower()

        outgoing_str = self.outgoing_sysid_var.get()
        outgoing_sysid = int(outgoing_str) if outgoing_str else None

        if msg_type.startswith(COMMAND_FILTER_PREFIX):
            command_name = msg_type[len(COMMAND_FILTER_PREFIX):]
            entries = [e for e in data.messages if e["fields"].get("command_name") == command_name]
            per_type_columns = False
        elif msg_type and msg_type != "All":
            entries = data.by_type.get(msg_type, [])
            per_type_columns = True
        else:
            entries = data.messages
            per_type_columns = False

        if per_type_columns:
            sample_fields = list(entries[0]["fields"].keys()) if entries else []
            columns = ["time", "sysid"] + sample_fields
        else:
            columns = ["time", "sysid", "type", "fields"]

        if search:
            def matches(e):
                if search in e["type"].lower():
                    return True
                return any(search in str(v).lower() for v in e["fields"].values())
            entries = [e for e in entries if matches(e)]

        def direction_of(e):
            if outgoing_sysid is None:
                return "?"
            return "OUT" if e["sysid"] == outgoing_sysid else "IN"

        direction_filter = self.direction_var.get()
        if direction_filter == "Outgoing":
            entries = [e for e in entries if direction_of(e) == "OUT"]
        elif direction_filter == "Incoming":
            entries = [e for e in entries if direction_of(e) == "IN"]

        if self.sort_column in columns:
            def sort_key(e):
                if self.sort_column == "time":
                    return e["time"]
                if self.sort_column == "sysid":
                    return e["sysid"]
                if self.sort_column == "type":
                    return e["type"]
                if self.sort_column == "fields":
                    return str(e["fields"])
                return e["fields"].get(self.sort_column, "")
            try:
                entries = sorted(entries, key=sort_key, reverse=self.sort_reverse)
            except TypeError:
                entries = sorted(entries, key=lambda e: str(sort_key(e)), reverse=self.sort_reverse)
        else:
            self.sort_column = None

        shown = entries[:MAX_TABLE_ROWS]
        rows = []
        for e in shown:
            t = datetime.fromtimestamp(e["time"]).strftime("%H:%M:%S.%f")[:-3]
            direction = direction_of(e)
            tag = ("outgoing",) if direction == "OUT" else ("incoming",) if direction == "IN" else ()

            if per_type_columns:
                row = [t, e["sysid"]] + [e["fields"].get(c, "") for c in columns[2:]]
            else:
                row = [t, e["sysid"], e["type"], str(e["fields"])]
            rows.append((row, tag))

        self.tree.delete(*self.tree.get_children())
        self.tree["columns"] = columns
        col_widths = self._compute_column_widths(columns, rows)
        for c, width in zip(columns, col_widths):
            heading = c
            if c == self.sort_column:
                heading += " ▼" if self.sort_reverse else " ▲"
            self.tree.heading(c, text=heading, command=lambda c=c: self._on_header_click(c))
            # stretch=False is required for both an accurate minimal width
            # and for horizontal scrolling to do anything: with the ttk
            # default (stretch=True), Treeview silently re-stretches every
            # column to exactly fill the visible frame width whenever the
            # computed total is narrower than that, which both widens
            # columns past their content and leaves nothing for the
            # scrollbar/Shift+wheel to scroll into.
            self.tree.column(c, width=width, minwidth=MIN_COLUMN_WIDTH, anchor="w", stretch=False)

        for row, tag in rows:
            self.tree.insert("", "end", values=row, tags=tag)

        note = "" if len(entries) <= MAX_TABLE_ROWS else f" (showing first {MAX_TABLE_ROWS})"
        self.count_label.config(text=f"{len(entries)} messages{note}")

    def _compute_column_widths(self, columns, rows):
        """Exact width per column that fits its header and every currently
        shown row's text for that column (floored at MIN_COLUMN_WIDTH)."""
        font = tkfont.nametofont("TkDefaultFont")
        widths = []
        for i, c in enumerate(columns):
            width = font.measure(str(c)) + COLUMN_HEADER_PAD
            for row, _tag in rows:
                cell_width = font.measure(str(row[i])) + COLUMN_CELL_PAD
                if cell_width > width:
                    width = cell_width
            widths.append(max(width, MIN_COLUMN_WIDTH))
        return widths

    def _on_header_click(self, col):
        if self.sort_column == col:
            self.sort_reverse = not self.sort_reverse
        else:
            self.sort_column = col
            self.sort_reverse = False
        self.apply_filter()

    def _show_context_menu(self, event):
        row_id = self.tree.identify_row(event.y)
        if row_id and row_id not in self.tree.selection():
            self.tree.selection_set(row_id)
        self._context_click_pos = (event.x, event.y)
        self.context_menu.tk_popup(event.x_root, event.y_root)

    def copy_selected_rows(self, event=None):
        selected = self.tree.selection()
        if not selected:
            return "break"
        columns = self.tree["columns"]
        lines = ["\t".join(columns)]
        for iid in selected:
            values = self.tree.item(iid, "values")
            lines.append("\t".join(str(v) for v in values))
        self.clipboard_clear()
        self.clipboard_append("\n".join(lines))
        return "break"

    def copy_clicked_cell(self):
        if not self._context_click_pos:
            return
        x, y = self._context_click_pos
        row_id = self.tree.identify_row(y)
        col_id = self.tree.identify_column(x)
        if not row_id or not col_id:
            return
        col_index = int(col_id.replace("#", "")) - 1
        values = self.tree.item(row_id, "values")
        if 0 <= col_index < len(values):
            self.clipboard_clear()
            self.clipboard_append(str(values[col_index]))


class PlotTab(ttk.Frame):
    """Build up a list of type.field series and plot them over time."""

    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self.series = []  # list of (msg_type, field)
        self._build()

    def _build(self):
        controls = ttk.Frame(self, padding=5)
        controls.pack(side="top", fill="x")

        ttk.Label(controls, text="Message type:").pack(side="left")
        self.type_var = tk.StringVar()
        self.type_combo = ttk.Combobox(controls, textvariable=self.type_var, state="readonly", width=20)
        self.type_combo.pack(side="left", padx=5)
        self.type_combo.bind("<<ComboboxSelected>>", lambda e: self._update_fields())

        ttk.Label(controls, text="Field:").pack(side="left", padx=(15, 0))
        self.field_var = tk.StringVar()
        self.field_combo = ttk.Combobox(controls, textvariable=self.field_var, state="readonly", width=20)
        self.field_combo.pack(side="left", padx=5)

        ttk.Button(controls, text="Add series", command=self.add_series).pack(side="left", padx=10)
        ttk.Button(controls, text="Remove selected", command=self.remove_series).pack(side="left")
        ttk.Button(controls, text="Plot", command=self.plot).pack(side="left", padx=10)
        ttk.Button(controls, text="Clear plot", command=self.clear_plot).pack(side="left")

        body = ttk.Frame(self)
        body.pack(fill="both", expand=True)

        left = ttk.Frame(body, padding=5)
        left.pack(side="left", fill="y")
        ttk.Label(left, text="Series to plot:").pack(anchor="w")
        self.series_list = tk.Listbox(left, width=32, height=20, exportselection=False)
        self.series_list.pack(fill="y", expand=True)

        right = ttk.Frame(body)
        right.pack(side="left", fill="both", expand=True)

        self.figure = Figure(figsize=(6, 4), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        toolbar = NavigationToolbar2Tk(self.canvas, right)
        toolbar.update()

    def on_data_loaded(self):
        types = sorted(t for t, fields in self.app.data.numeric_fields.items() if fields)
        self.type_combo["values"] = types
        if types:
            self.type_var.set(types[0])
            self._update_fields()

    def _update_fields(self):
        msg_type = self.type_var.get()
        fields = self.app.data.numeric_fields.get(msg_type, [])
        self.field_combo["values"] = fields
        if fields:
            self.field_var.set(fields[0])

    def add_series(self):
        msg_type = self.type_var.get()
        field = self.field_var.get()
        if not msg_type or not field:
            return
        key = (msg_type, field)
        if key in self.series:
            return
        self.series.append(key)
        self.series_list.insert("end", f"{msg_type}.{field}")

    def remove_series(self):
        for idx in reversed(self.series_list.curselection()):
            self.series_list.delete(idx)
            del self.series[idx]

    def clear_plot(self):
        self.series = []
        self.series_list.delete(0, "end")
        self.ax.clear()
        self.canvas.draw()

    def plot(self):
        if not self.series:
            messagebox.showinfo("Plot", "Add at least one series first.")
            return

        self.ax.clear()
        data = self.app.data
        for msg_type, field in self.series:
            entries = data.by_type.get(msg_type, [])
            times = [datetime.fromtimestamp(e["time"]) for e in entries if field in e["fields"]]
            values = [e["fields"][field] for e in entries if field in e["fields"]]
            self.ax.plot(times, values, label=f"{msg_type}.{field}", linewidth=0.8)

        self.ax.legend(loc="upper right", fontsize=8)
        self.ax.set_xlabel("Time")
        self.ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
        self.figure.autofmt_xdate()
        self.ax.grid(True, alpha=0.3)
        self.canvas.draw()


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ArduPilot Tlog Viewer")
        self.geometry("1200x800")
        self.data = TlogData()

        self._build_toolbar()
        self._build_notebook()
        self._build_statusbar()

    def _build_toolbar(self):
        bar = ttk.Frame(self, padding=5)
        bar.pack(side="top", fill="x")
        ttk.Button(bar, text="Open tlog...", command=self.open_file).pack(side="left")
        self.file_label = ttk.Label(bar, text="No file loaded")
        self.file_label.pack(side="left", padx=10)

    def _build_statusbar(self):
        self.status = ttk.Label(self, text="Ready", anchor="w", relief="sunken")
        self.status.pack(side="bottom", fill="x")

    def _build_notebook(self):
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True)
        self.messages_tab = MessagesTab(nb, self)
        self.plot_tab = PlotTab(nb, self)
        nb.add(self.messages_tab, text="Messages")
        nb.add(self.plot_tab, text="Plot")

    def open_file(self):
        path = filedialog.askopenfilename(
            title="Open tlog file",
            filetypes=[("Telemetry logs", "*.tlog *.tlog.raw"), ("All files", "*.*")],
        )
        if not path:
            return

        self.file_label.config(text=os.path.basename(path))
        self.status.config(text="Loading...")

        def worker():
            data = TlogData()
            try:
                count = data.load(
                    path,
                    progress_cb=lambda c: self.after(0, lambda: self.status.config(text=f"Loaded {c} messages...")),
                )
            except Exception as exc:
                self.after(0, lambda: messagebox.showerror("Error loading tlog", str(exc)))
                self.after(0, lambda: self.status.config(text="Error"))
                return
            self.after(0, lambda: self._on_loaded(data, count))

        threading.Thread(target=worker, daemon=True).start()

    def _on_loaded(self, data, count):
        self.data = data
        span = ""
        if data.start_time and data.end_time:
            duration = data.end_time - data.start_time
            span = f" | duration: {duration:.1f}s | start: {datetime.fromtimestamp(data.start_time)}"
        self.status.config(text=f"Loaded {count} messages, {len(data.by_type)} types{span}")
        self.messages_tab.on_data_loaded()
        self.plot_tab.on_data_loaded()


def main():
    app = App()
    app.mainloop()


if __name__ == "__main__":
    main()
