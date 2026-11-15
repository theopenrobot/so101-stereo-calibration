"""Preview relative leader targets. Never writes registers or enables torque."""

import argparse
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import runpy
import subprocess
import time

import sys as _sys; _sys.path.insert(0, str(Path(__file__).resolve().parent)); import so101_host as HOST   # Mac / Linux differences: ports, speech, busy-port check


JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def next_targets(leader_start, leader_now, follower_start, previous, limits, dt, speed=80, travel=None):
    """speed: ticks per second. travel: largest distance from the follower start, in ticks."""
    step = int(speed * min(dt, 0.1))
    targets = {}
    for name in JOINTS:
        delta = leader_now[name] - leader_start[name]
        if name == "wrist_roll":
            delta = (delta + 2048) % 4096 - 2048
        low, high = limits[name]
        desired = max(low, min(high, follower_start[name] + delta))
        if travel is not None:
            desired = max(follower_start[name] - travel, min(follower_start[name] + travel, desired))
        targets[name] = previous[name] + max(-step, min(step, desired - previous[name]))
    return targets


def self_test():
    leader = dict(zip(JOINTS, (1000, 1500, 2000, 2500, 4090)))
    follower = dict(zip(JOINTS, (2000, 1600, 1800, 2200, 2048)))
    leader["gripper"] = 1234
    limits = {name: (500, 3500) for name in JOINTS}
    limits["wrist_roll"] = (32, 4063)
    assert next_targets(leader, leader, follower, follower, limits, 0.1) == follower
    moved = {**leader, "shoulder_pan": 1090, "shoulder_lift": 1440, "wrist_flex": 2525, "wrist_roll": 6}
    expected = dict(zip(JOINTS, (2008, 1592, 1800, 2208, 2056)))
    actual = next_targets(leader, moved, follower, follower, limits, 0.1)
    assert actual == expected, actual
    assert "gripper" not in actual
    assert next_targets(leader, moved, follower, follower, limits, 2.0) == expected
    assert next_targets(leader, moved, follower, follower, limits, 0.05) == dict(
        zip(JOINTS, (2004, 1596, 1800, 2204, 2052))
    )
    edge = {**follower, "shoulder_pan": 3498, "shoulder_lift": 501}
    assert next_targets(leader, moved, edge, edge, limits, 0.1) == dict(
        zip(JOINTS, (3500, 500, 1800, 2208, 2056))
    )
    rest = {**follower, "elbow_flex": 3510, "wrist_flex": 470}
    assert next_targets(leader, leader, rest, rest, limits, 0.1) == dict(
        zip(JOINTS, (2000, 1600, 3502, 478, 2048))
    )


def preview(seconds):
    root = Path(__file__).resolve().parent
    safe = runpy.run_path(str(root / "so101-calibrate-safe.py"))

    class ReadOnlyBus(safe["FeetechMotorsBus"]):
        def write(self, *args, **kwargs):
            raise RuntimeError("Preview only: register writes blocked")

        def sync_write(self, *args, **kwargs):
            raise RuntimeError("Preview only: motor targets blocked")

    leader = safe["SO101Leader"](safe["SO101LeaderConfig"](port=HOST.leader_port(), id="leader"))
    follower = safe["SO101Follower"](safe["SO101FollowerConfig"](port=HOST.follower_port(), id="follower"))
    profile = HOST.profile_path()
    expected = {
        "leader": {name: asdict(leader.calibration[name]) for name in JOINTS},
        "follower": json.loads(profile.read_text()),
    }
    buses = {
        key: ReadOnlyBus(device.bus.port, {name: device.bus.motors[name] for name in JOINTS})
        for key, device in (("leader", leader), ("follower", follower))
    }
    rows = []
    try:
        for key, bus in buses.items():
            users = HOST.port_users(bus.port)
            if users.returncode not in (0, 1) or users.stdout.strip():
                raise RuntimeError(f"{key} serial port unavailable")
            bus.connect(handshake=False)
            if any(bus.ping(name) != 777 for name in bus.motors):
                raise RuntimeError(f"{key}: motor replies missing; preview cannot start")
            if any(bus.read("Torque_Enable", name) for name in bus.motors):
                raise RuntimeError(f"{key}: torque must already be off; preview will not change it")
            actual = bus.read_calibration()
            if any(asdict(actual[name]) != expected[key][name] for name in JOINTS):
                raise RuntimeError(f"{key}: saved five-joint calibration does not match motors")
        limits = {
            name: (entry["range_min"] + 32, entry["range_max"] - 32)
            for name, entry in expected["follower"].items()
        }
        leader_start = buses["leader"].sync_read("Present_Position", JOINTS, normalize=False, num_retry=2)
        follower_start = buses["follower"].sync_read("Present_Position", JOINTS, normalize=False, num_retry=2)
        for name in JOINTS:
            entry = expected["follower"][name]
            if not entry["range_min"] - 64 <= follower_start[name] <= entry["range_max"] + 64:
                raise RuntimeError(f"Follower {name} is outside its recorded range; calibration does not fit this pose")
        previous = follower_start
        start = last = time.monotonic()
        print("PREVIEW ONLY. Move the black leader gently. White will NOT move.", flush=True)
        while time.monotonic() - start < seconds:
            tick = time.monotonic()
            leader_now = buses["leader"].sync_read("Present_Position", JOINTS, normalize=False, num_retry=2)
            follower_now = buses["follower"].sync_read("Present_Position", JOINTS, normalize=False, num_retry=2)
            now = time.monotonic()
            if now - last > 0.25:
                raise RuntimeError("Position reads delayed more than 250 ms; ending preview")
            targets = next_targets(leader_start, leader_now, follower_start, previous, limits, now - last)
            row = {
                "elapsed_seconds": now - start,
                "leader_raw": leader_now,
                "follower_raw": follower_now,
                "unsent_targets_raw": targets,
            }
            rows.append(row)
            print(json.dumps(row), flush=True)
            previous, last = targets, now
            time.sleep(max(0, 0.1 - (time.monotonic() - tick)))
    finally:
        for bus in buses.values():
            if bus.is_connected:
                bus.disconnect(disable_torque=False)
        if rows:
            path = root / f"so101-target-preview-{datetime.now():%Y%m%d-%H%M%S-%f}.json"
            path.write_text(json.dumps({
                "purpose": "UNSENT target preview, not a training dataset or demonstrated actions",
                "motor_writes": 0, "rows": rows,
            }, indent=2))
            print(f"Preview evidence: {path}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true", help="Offline checks; never opens hardware")
    parser.add_argument("--seconds", type=float, default=10)
    args = parser.parse_args()
    if args.self_test:
        self_test()
        print("PASS: relative start, wrist wrap, five-joint exclusion, bounds and step caps. No hardware opened.")
        return
    if not 0 < args.seconds <= 300:
        parser.error("--seconds must be greater than zero and at most 300")
    preview(args.seconds)


if __name__ == "__main__":
    main()
