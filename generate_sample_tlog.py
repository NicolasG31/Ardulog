"""Generate a synthetic ArduPilot-style .tlog for testing/demo purposes.

Simulates a short flight (climb, circuit cruise, descent) across several
common ArduPilot MAVLink message types, so tlog_viewer.py has realistic,
varied data to filter, search, and plot without needing a real flight log.

pymavlink's mavlogfile does not prepend the 8-byte timestamp header on
write (only on read), so we wrap the output file ourselves. See CLAUDE.md
for details.
"""
import math
import struct
import time

from pymavlink import mavutil

mavutil.set_dialect("ardupilotmega")
mavutil.mavlink20()  # write MAVLink 2 frames, matching what mavlink_connection reads by default

OUTPUT_PATH = "sample_flight.tlog"
DURATION_S = 120.0
HOME_LAT = -35.363261
HOME_LON = 149.165230


class TimestampedFile:
    """Prefixes every MAVLink packet write with the 8-byte big-endian
    microsecond timestamp header the .tlog format requires."""

    def __init__(self, path, start_time):
        self.f = open(path, "wb")
        self.t = start_time

    def write(self, buf):
        self.f.write(struct.pack(">Q", int(self.t * 1e6)))
        self.f.write(buf)

    def close(self):
        self.f.close()


def flight_profile(t):
    """Return (alt_m, roll_rad, pitch_rad, yaw_rad, lat, lon, groundspeed_ms)
    for simulated time t (seconds since takeoff)."""
    if t < 20:  # climb
        alt = 50.0 * (t / 20)
        roll = 0.05 * math.sin(t)
        pitch = math.radians(10)
        yaw = 0.0
        gs = 8.0 * (t / 20)
    elif t < 90:  # cruise circuit
        alt = 50.0 + 2.0 * math.sin(t / 5)
        turn_t = t - 20
        yaw = (turn_t / 70) * 2 * math.pi
        roll = math.radians(15) * math.sin(turn_t / 3)
        pitch = math.radians(2) * math.sin(turn_t / 4)
        gs = 12.0
    else:  # descent
        dt = t - 90
        alt = max(0.0, 50.0 * (1 - dt / 30))
        roll = 0.02 * math.sin(t)
        pitch = math.radians(-5)
        yaw = 2 * math.pi
        gs = max(0.0, 12.0 * (1 - dt / 30))

    radius_deg = 0.001
    lat = HOME_LAT + radius_deg * math.sin(yaw)
    lon = HOME_LON + radius_deg * math.cos(yaw)
    return alt, roll, pitch, yaw % (2 * math.pi), lat, lon, gs


def flight_mode(t):
    """ArduCopter custom_mode number for simulated time t, matching the
    climb/cruise/descent phases in flight_profile and the gcs_commands arm/
    takeoff/land timings below, so HEARTBEAT's mode field tells a coherent
    story (STABILIZE on the ground -> GUIDED climb -> AUTO cruise -> RTL ->
    LAND)."""
    if t < 3:
        return 0   # STABILIZE, armed on the ground before takeoff
    if t < 20:
        return 4   # GUIDED, climbing out
    if t < 90:
        return 3   # AUTO, cruising the circuit
    if t < 110:
        return 6   # RTL, heading home
    return 9       # LAND


def main():
    start_time = time.time() - DURATION_S  # log appears to end "now"
    tsf = TimestampedFile(OUTPUT_PATH, start_time)
    # Vehicle side (sysid 1) — telemetry stream.
    mav = mavutil.mavlink.MAVLink(tsf, srcSystem=1, srcComponent=1)
    # GCS side (sysid 255) — commands/requests, so incoming vs outgoing
    # coloring in the viewer has both directions to show.
    mav_gcs = mavutil.mavlink.MAVLink(tsf, srcSystem=255, srcComponent=0)

    # A few PARAM_VALUE messages up front, like a param stream at connect.
    params = [
        ("WPNAV_SPEED", 500.0),
        ("ANGLE_MAX", 3000.0),
        ("BATT_LOW_VOLT", 10.5),
        ("FENCE_ENABLE", 1.0),
        ("RTL_ALT", 1500.0),
    ]
    tsf.t = start_time
    mav_gcs.param_request_list_send(1, 1)
    for i, (name, value) in enumerate(params):
        mav.param_value_send(
            name.encode("utf-8"), value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32, len(params), i,
        )

    # One-off GCS commands at fixed simulated-flight times.
    gcs_commands = {
        1.0: lambda: mav_gcs.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
        ),
        3.0: lambda: mav_gcs.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 50
        ),
        110.0: lambda: mav_gcs.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0
        ),
    }

    periods = {
        "heartbeat": 1.0,
        "gcs_heartbeat": 1.0,
        "sys_status": 1.0,
        "battery": 1.0,
        "gps": 0.2,
        "attitude": 0.1,
        "vfr": 0.1,
    }
    next_due = {k: 0.0 for k in periods}

    voltage_start, voltage_end = 12600, 11200  # mV, simulated discharge

    t = 0.0
    step = 0.02
    while t <= DURATION_S:
        tsf.t = start_time + t
        alt, roll, pitch, yaw, lat, lon, gs = flight_profile(t)
        alt_mm = int(alt * 1000)
        lat_e7 = int(lat * 1e7)
        lon_e7 = int(lon * 1e7)
        boot_ms = int(t * 1000)
        heading_cd = int(math.degrees(yaw) * 100) % 36000
        voltage = int(voltage_start + (voltage_end - voltage_start) * (t / DURATION_S))

        if t >= next_due["heartbeat"]:
            base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            if t >= 1.0:  # armed by the ARM_DISARM command below
                base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                base_mode, flight_mode(t), mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            next_due["heartbeat"] += periods["heartbeat"]

        if t >= next_due["gcs_heartbeat"]:
            mav_gcs.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            next_due["gcs_heartbeat"] += periods["gcs_heartbeat"]

        for cmd_t in [k for k in gcs_commands if k <= t]:
            gcs_commands.pop(cmd_t)()

        if t >= next_due["sys_status"]:
            mav.sys_status_send(0, 0, 0, 20, voltage, 3500, 80, 0, 0, 0, 0, 0, 0)
            next_due["sys_status"] += periods["sys_status"]

        if t >= next_due["battery"]:
            mav.battery_status_send(
                0, 0, 0, 2500, [voltage] + [65535] * 9,
                3500, -1, -1, 60,
            )
            next_due["battery"] += periods["battery"]

        if t >= next_due["gps"]:
            mav.gps_raw_int_send(
                boot_ms * 1000, 3, lat_e7, lon_e7, alt_mm,
                100, 100, int(gs * 100), heading_cd, 12,
            )
            mav.global_position_int_send(
                boot_ms, lat_e7, lon_e7, alt_mm, alt_mm - 2000,
                0, 0, 0, heading_cd,
            )
            next_due["gps"] += periods["gps"]

        if t >= next_due["attitude"]:
            mav.attitude_send(boot_ms, roll, pitch, yaw, 0.0, 0.0, 0.0)
            next_due["attitude"] += periods["attitude"]

        if t >= next_due["vfr"]:
            mav.vfr_hud_send(gs, gs, int(math.degrees(yaw)) % 360, 50, alt, 0.0)
            next_due["vfr"] += periods["vfr"]

        t += step

    tsf.close()
    print(f"Wrote {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
