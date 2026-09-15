# Ardulog Tlog Viewer

A small desktop app for decoding and inspecting ArduPilot MAVLink telemetry
logs (`.tlog`). Browse and search decoded messages, and plot numeric fields
over time — including comparing fields from different message types on the
same chart.

## Requirements

- Python 3.9+
- `pip install -r requirements.txt` (installs `pymavlink` and `matplotlib`)

## Running

```
python tlog_viewer.py
```

## Try it with sample data

No real tlog handy? Generate a synthetic one:

```
python generate_sample_tlog.py
```

This writes `sample_flight.tlog` — a simulated ~2 minute flight (climb,
circuit cruise, descent) with two simulated sysids: the vehicle (1, sending
`HEARTBEAT`, `SYS_STATUS`, `BATTERY_STATUS`, `GPS_RAW_INT`,
`GLOBAL_POSITION_INT`, `ATTITUDE`, `VFR_HUD`, `PARAM_VALUE`) and a GCS
(255, sending its own `HEARTBEAT`, `PARAM_REQUEST_LIST`, and a few
`COMMAND_LONG` — arm, takeoff, land) — enough variety to try filtering,
searching, plotting (e.g. add `ATTITUDE.roll` and `ATTITUDE.pitch` as
series, or compare `GLOBAL_POSITION_INT.alt` against `VFR_HUD.alt`), and
the incoming/outgoing filtering/coloring described below. The vehicle's
`HEARTBEAT` mode also walks through a coherent flight: `STABILIZE` (armed,
on the ground) → `GUIDED` (climb) → `AUTO` (cruise) → `RTL` → `LAND`.
Open it from the app with **Open tlog...**.

## Features

- **Open tlog...** — load a `.tlog` file. Loading runs in the background so
  the window stays responsive; the status bar shows the message count,
  number of distinct message types, and time span once done.
- **Messages tab**
  - Filter the table by message type (dropdown, populated from what's in
    the loaded log).
  - Free-text search across all field values (case-insensitive substring
    match), combinable with the type filter.
  - Table columns adapt to the selected message type's fields; the `type`
    column (shown in the "All" view) is widened so full message type names
    are readable without resizing.
  - **Incoming/outgoing coloring and filtering** — rows are colored by
    direction, based on which sysid you tell it is "yours" via the
    **Outgoing sysid** dropdown (auto-guessed on load: prefers a
    `HEARTBEAT` explicitly typed `MAV_TYPE_GCS`, otherwise the higher of
    exactly two sysids seen). Change the dropdown if the guess is wrong.
    The **Direction** dropdown (All/Outgoing/Incoming) filters the table
    down to just one direction, combinable with the type filter and
    search.
  - **Flight mode as text** — `HEARTBEAT` rows get a `mode` column showing
    the decoded flight mode name (e.g. `STABILIZE`, `AUTO`, `RTL`) instead
    of just the raw `custom_mode` number; it's also searchable like any
    other field.
  - **Command names as text** — any row with a `command` field
    (`COMMAND_LONG`, `COMMAND_INT`, `COMMAND_ACK`, `MISSION_ITEM`, ...)
    gets a `command_name` column showing the decoded `MAV_CMD_*` name
    (e.g. `MAV_CMD_NAV_TAKEOFF`) instead of just the raw numeric id;
    filter to a command type and/or search for a command name to find
    specific commands.
  - **Sort by clicking a column header** — click again to reverse. Sorting
    combines with whatever filters are active.
  - **Copy values** — select row(s) and press Ctrl+C, or right-click for
    **Copy row(s)** (tab-separated, with a header row) / **Copy cell**
    (right-click near the cell you want) — pastes straight into a
    spreadsheet or text editor.
- **Plot tab**
  - Pick a message type and a numeric field, then **Add series**.
  - Add as many series as you like, from any message types (e.g. compare
    `ATTITUDE.roll` against `AHRS2.roll`, or `GPS_RAW_INT.alt` against
    `BARO.altitude`).
  - **Plot** draws all added series on one time-synced chart with
    pan/zoom/save (standard matplotlib toolbar).
  - **Remove selected** / **Clear plot** to manage the series list.

## Notes / limitations

- Assumes ArduPilot's MAVLink dialect (`ardupilotmega`) — not currently
  configurable.
- The message table displays at most 20,000 rows at a time for
  performance; narrow with the type filter or search to see everything for
  a specific type on very large logs.
- One tlog loaded at a time — no multi-file comparison yet.
- No PARAM_VALUE diffing yet.

## Roadmap ideas

- Compare PARAM_VALUE messages across two logs or two time points.
- Export a filtered message view to CSV.
- Multi-log / session comparison.

---
*This README is kept in sync with `tlog_viewer.py` — see `CLAUDE.md` for
implementation notes and the maintenance rule.*
