# Ardulog — ArduPilot Tlog Viewer

## What this project is

A single-file Tkinter GUI (`tlog_viewer.py`) that loads ArduPilot MAVLink
telemetry logs (`.tlog`) and lets the user browse/search decoded messages
and plot numeric fields over time (including comparing fields from
different message types on one chart).

Built for a GCS developer who generates tlogs from GCS↔drone MAVLink
traffic and needs to inspect them locally — not a MAVProxy/QGC replacement,
just a fast decode-and-look tool.

## Current structure

- `tlog_viewer.py` — the whole app. Three parts:
  - `TlogData` — decodes a tlog via `pymavlink.mavutil.mavlink_connection`,
    dialect hardcoded to `ardupilotmega`. Builds `messages` (time-sorted
    list), `by_type` (dict of type → entries), `numeric_fields` (dict of
    type → sorted numeric field names, computed from the first sample of
    each type — assumes a type's fields are homogeneous across a log).
  - `MessagesTab` — Treeview table, filter by message type + free-text
    search across all field values (substring, case-insensitive). Caps
    display at `MAX_TABLE_ROWS = 20000` for perf; full data is still in
    memory, just not rendered. Rows are colored by direction (`outgoing`/
    `incoming` Treeview tags → `OUTGOING_BG`/`INCOMING_BG`) based on
    comparing each entry's `sysid` to the **Outgoing sysid** dropdown
    (`outgoing_sysid_var`); a `dir`/`sysid` column pair also renders this
    as text so it's not color-only. Ctrl+C or right-click → **Copy row(s)**
    copies the current Treeview selection as tab-separated text (header +
    rows) to the clipboard; right-click → **Copy cell** copies just the
    cell under the click (via `tree.identify_row`/`identify_column`).
  - `PlotTab` — pick type+field, "Add series" to a list, "Plot" draws all
    added series on one matplotlib chart (`FigureCanvasTkAgg` + nav
    toolbar), x-axis is wall-clock time from each message's `_timestamp`.
  - `App` — top-level window, wires the two tabs into a `ttk.Notebook`.
    File loading runs in a background thread (`threading.Thread`) so the
    UI doesn't freeze on large logs; result is marshaled back via
    `self.after(0, ...)`.
- `generate_sample_tlog.py` — writes `sample_flight.tlog`, a synthetic
  ~2 minute flight (climb/cruise/descent) with two simulated sysids: the
  vehicle (1: `HEARTBEAT`, `SYS_STATUS`, `BATTERY_STATUS`, `GPS_RAW_INT`,
  `GLOBAL_POSITION_INT`, `ATTITUDE`, `VFR_HUD`, `PARAM_VALUE`) and a GCS
  (255: `HEARTBEAT` typed `MAV_TYPE_GCS`, `PARAM_REQUEST_LIST`, a few
  `COMMAND_LONG`) — the two sysids exist specifically so the
  incoming/outgoing coloring feature has real bidirectional data to show.
  Used for demoing/testing the viewer without a real flight log; contains
  the canonical `TimestampedFile` wrapper (see below) — reuse it rather
  than re-deriving.
- `sample_flight.tlog` — generated output, not hand-maintained; regenerate
  with `python generate_sample_tlog.py` if the generator changes.
- `requirements.txt` — `pymavlink`, `matplotlib`.
- `README.md` — user-facing usage doc. **Keep it in sync** — see below.

## Design decisions / why

- **Tkinter, not PyQt/Streamlit** — user picked it explicitly: no extra
  heavy dependency, native desktop window, ships with Python.
- **Two tabs (Messages + Plot), no PARAM diffing tab** — user's answer to
  the initial scoping question. PARAM diffing was offered but not chosen;
  it's a natural next tab if requested (compare `PARAM_VALUE` messages
  between two logs or two time windows).
- **Dialect hardcoded to `ardupilotmega`** — correct default for ArduPilot
  logs; not exposed as a UI option since the user's use case is always
  ArduPilot. Revisit if PX4 or another dialect ever comes up.
- **Numeric field list built from the first message of each type** — cheap
  and works for MAVLink since a message type's schema is fixed, so no need
  to scan every message.
- **`by_type` entries are the same dict objects as in `messages`** (not
  copies) — intentional, keeps memory down for large logs.
- **Direction (incoming/outgoing) is inferred from sysid, not stored in
  the tlog** — tlogs don't record link direction, only each message's
  source system id (`msg.get_srcSystem()`). `TlogData._guess_outgoing_sysid`
  picks a default (prefers a `HEARTBEAT` typed `MAV_TYPE_GCS`; falls back
  to the higher of exactly two sysids, since ArduPilot vehicles default to
  sysid 1 and GCS software conventionally defaults to 255) but it's just a
  starting point — the user can override it via the dropdown, since a tlog
  could have >2 sysids (multiple vehicles/GCSes) or use non-default ids.

## How synthetic tlogs are generated for testing

`pymavlink.mavutil.mavlogfile` does **not** prepend the 8-byte big-endian
microsecond timestamp header on write — that's only handled on read
(`pre_message`/`scan_timestamp`). `generate_sample_tlog.py`'s
`TimestampedFile` wrapper handles this: every `write()` call (one per
MAVLink packet) is preceded by `struct.pack('>Q', int(timestamp_usec))`.

Also required: call `mavutil.mavlink20()` before constructing the
`MAVLink` writer. Without it, `mavutil.mavlink` defaults to the MAVLink 1
dialect module, whose message field layouts (e.g. `BATTERY_STATUS`) differ
from the v2 module `mavlink_connection` reads with by default — this
caused a `struct.error` on `voltages` (v1 has no `-1`-as-unused sentinel
support; had to use `65535` for unset `uint16` voltage slots) and would
otherwise risk CRC/field mismatches on read.

Verified the full load → filter → search → multi-series plot pipeline
headlessly this way: construct `App()`, call `app._on_loaded(data, count)`
directly, inspect `tree.get_children()` / `ax.lines` — no display needed
beyond what Tkinter requires to construct widgets.

## Known limitations / backlog

- No PARAM_VALUE diffing between logs or time points (not requested yet).
- Message table hard-caps at 20k displayed rows; no virtualization/paging,
  so "All types, no filter" on a huge log only shows the first slice.
- No log export (e.g. filtered view → CSV).
- Dialect is not user-selectable.
- No multi-file / session comparison (only one tlog loaded at a time).

## Maintenance rule

**Whenever `tlog_viewer.py`'s features, UI, or behavior change, update
`README.md` in the same change** — its "Features" and "Usage" sections
should always describe the current app, not a past version. Update this
file too if structure/design decisions materially change.
