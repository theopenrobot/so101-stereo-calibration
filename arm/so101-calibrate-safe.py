"""Torque-off SO-101 calibration with settled readings and bounded register writes."""

import argparse
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import shutil
import time

import sys as _sys; _sys.path.insert(0, str(Path(__file__).resolve().parent)); import so101_host as HOST   # Mac / Linux differences: ports, speech, busy-port check

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig


def validate_calibration(calibration):
    for name, value in calibration.items():
        if not 0 <= value.range_min < value.range_max <= 4095:
            raise ValueError(f"Refusing invalid limits for {name}: {value}")
        if not -2047 <= value.homing_offset <= 2047:
            raise ValueError(f"Refusing invalid homing offset for {name}: {value}")


class CalibrationBus(FeetechMotorsBus):
    def write(self, data_name, motor, value, **kwargs):
        allowed = {
            "Torque_Enable": (0, 0),
            "Lock": (0, 0),
            "Operating_Mode": (0, 0),
            "Homing_Offset": (-2047, 2047),
            "Min_Position_Limit": (0, 4095),
            "Max_Position_Limit": (0, 4095),
        }
        bounds = allowed.get(data_name)
        if bounds is None or not bounds[0] <= value <= bounds[1]:
            raise ValueError(f"Calibration-only write rejected: {motor} {data_name}={value}")
        return super().write(data_name, motor, value, **kwargs)

    def require_torque_off(self):
        states = {name: self.read("Torque_Enable", name) for name in self.motors}
        print("TORQUE READBACK:", states, flush=True)
        if any(states.values()):
            raise RuntimeError("Torque is not off. No calibration will start.")

    def set_half_turn_homings(self, motors=None):
        names = self._get_motors_list(motors)
        self.require_torque_off()
        self.reset_calibration(names)
        time.sleep(0.3)
        positions = self.sync_read("Present_Position", names, normalize=False)
        offsets = self._get_half_turn_homings(positions)
        print("RAW MIDPOINT:", positions, "OFFSETS:", offsets, flush=True)
        if any(not -2047 <= offset <= 2047 for offset in offsets.values()):
            raise ValueError("Midpoint is outside one encoder turn. No homing offsets written.")
        for name, offset in offsets.items():
            self.write("Homing_Offset", name, offset)
        consecutive = 0
        for _ in range(30):
            time.sleep(0.1)
            positions = self.sync_read("Present_Position", names, normalize=False)
            consecutive = consecutive + 1 if all(abs(p - 2047) <= 3 for p in positions.values()) else 0
            if consecutive >= 3:
                print("CENTERED AND SETTLED:", positions, flush=True)
                return offsets
        raise RuntimeError(f"Midpoint readings did not settle. No ranges saved: {positions}")

    def write_calibration(self, calibration_dict, cache=True):
        validate_calibration(calibration_dict)
        self.require_torque_off()
        super().write_calibration(calibration_dict, cache=cache)
        actual = self.read_calibration()
        if actual != calibration_dict:
            raise RuntimeError(f"Motor calibration readback mismatch: {actual}")
        print("MOTOR CALIBRATION VERIFIED:", actual, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("arm", choices=("leader", "follower"))
    args = parser.parse_args()
    if args.arm == "leader":
        device = SO101Leader(SO101LeaderConfig(port=HOST.leader_port(), id="leader"))
    else:
        device = SO101Follower(SO101FollowerConfig(port=HOST.follower_port(), id="follower"))
    if device.calibration_fpath.exists():
        backup = device.calibration_fpath.with_suffix(f".{datetime.now():%Y%m%d-%H%M%S}.backup.json")
        shutil.copy2(device.calibration_fpath, backup)
        print("PREVIOUS CALIBRATION BACKUP:", backup, flush=True)
    device.calibration = {}
    device.bus = CalibrationBus(device.bus.port, device.bus.motors)
    try:
        device.bus.connect()
        device.bus.require_torque_off()
        print("TORQUE-OFF CALIBRATION ONLY. No camera access or movement commands.", flush=True)
        device.calibrate()
        saved = json.loads(device.calibration_fpath.read_text())
        expected = {name: asdict(value) for name, value in device.calibration.items()}
        if saved != expected:
            raise RuntimeError("Saved calibration JSON does not match verified motor values.")
        device.bus.require_torque_off()
        print("SAVED JSON AND MOTOR READBACK MATCH. TORQUE REMAINS OFF.", flush=True)
    finally:
        if device.bus.is_connected:
            device.bus.disconnect(disable_torque=False)
            print("SERIAL PORT RELEASED", flush=True)


if __name__ == "__main__":
    main()
