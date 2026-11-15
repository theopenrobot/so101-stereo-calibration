"""Five-joint SO-101 teleoperation: black leader to white follower, motor IDs 1 to 5 only.

Default is a DRY RUN: the full loop runs, but nothing is written to a motor.
--enable-torque sends torque and goal positions. Use it only with the operator's explicit approval.
The follower returns to its start pose before torque goes off. A fault turns torque off at once.

Energized time has a budget: enable + follow + return + release (see energized_budget). The software ends
each phase at its limit. This is not a hardware stop: if the program or the serial link stops, the servos
hold their last goal until the DC plug of the white controller board is pulled.

Preflight: --enable-torque needs --preflight <record.json>. The record states the measured clearance, the
operator readiness and an inspected camera frame, for this exact command (see preflight_refusals). A missing,
failed or old check stops the run before a serial port opens. There is no override. The record is a statement
by a person; it is not proof that the workspace is clear.

{"checked_at": "2025-01-01T12:00:10+00:00",
 "command": {"seconds": 20, "speed": 80, "travel": 100, "torque_limit": 500},
 "clearance": {"follower_reach_cm": 0, "leader_reach_cm": 0, "base_separation_cm": 0,
               "nearest_person_cm": 0, "margin_cm": 0, "method": "how each distance was measured"},
 "operator": {"ready": true, "confirmed_at": "2025-01-01T12:00:00+00:00"},
 "camera": {"frame": "/absolute/path.jpg", "sha256": "...", "captured_at": "2025-01-01T12:00:05+00:00",
            "shows_both_arms": true}}
"""

import argparse
from collections import Counter
import contextlib
from dataclasses import asdict, dataclass, replace
from datetime import datetime, timedelta
import hashlib
import io
from itertools import combinations
import json
import math
import os
from pathlib import Path
import runpy
import signal
import subprocess
import sys
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parent
import sys as _sys; _sys.path.insert(0, str(Path(__file__).resolve().parent)); import so101_host as HOST   # Mac / Linux differences: ports, speech, busy-port check
PREVIEW = runpy.run_path(str(ROOT / "so101-five-joint-preview.py"))
JOINTS, next_targets = PREVIEW["JOINTS"], PREVIEW["next_targets"]

PERIOD = 0.1  # seconds for each control cycle
RANGE_SLACK = 64  # ticks outside the recorded range that the start pose can be (the rest pose sits on the stops)
MARGIN = 32  # ticks kept free at each end of the recorded range
BAD_SAMPLES = 3  # consecutive samples of tracking error before torque goes off
HOME_TOLERANCE = 16  # ticks; the follower is "home" when all joints are this near the start
MAX_TEMPERATURE = 55  # degrees C
MAX_SECONDS = 300  # longest follow time; the health check (status, temperature) runs each second
RELEASE_PASSES = 5  # torque-off attempts; each attempt ends with a readback
RELEASE_SETTLE = 0.2  # seconds between the torque-off writes and the readback
ENABLE_BUDGET = 3.0  # seconds from the first write of the torque-on sequence to the start of the follow phase
RETURN_SETTLE = 3.0  # seconds added to travel / speed for the return phase
STILL_CYCLES, STILL_TICKS = 5, 2  # the arm is static when no joint moves more than 2 ticks in 5 cycles
RELEASE_BUDGET = 8.0  # seconds for the torque-off passes, the limit restore and the last readback
RELEASE_PASS_WINDOW = 3.0  # a new torque-off pass starts only in the first 3 seconds of the release
UNNAMED_FLAG = 16  # an error bit in a status packet that the Feetech SDK has no text for


PREFLIGHT_MAX_AGE = 180  # seconds from each check (record, operator, camera frame) to the torque-on sequence
REACH_FLOOR_CM = 47.8  # official SO-ARM100 URDF, pan joint origin to the tip frame; the installed tip adds to this
MIN_MARGIN_CM = 20.0  # free distance between the reach of the white arm and a person or the reach of the black arm
FRAME_MIN_SIZE = (640, 360)
FRAME_MIN_CONTRAST = 10.0  # standard deviation of the grey levels; a covered or dark lens is below this
COMMAND_KEYS = ("seconds", "speed", "travel", "torque_limit")


class PreflightRefused(RuntimeError):
    """The powered run does not start. Nothing was written to a motor."""


def frame_problems(camera, now):
    """Why the camera frame of the record is not usable evidence. The frame must be the file that was inspected."""
    try:
        path = Path(camera["frame"])
        if not path.is_absolute():
            return ["camera: frame must be an absolute path"]
        data, written = path.read_bytes(), now.timestamp() - path.stat().st_mtime
    except (KeyError, TypeError, OSError) as error:
        return [f"camera: the frame file is not readable ({error!r}). Take a new frame"]
    if hashlib.sha256(data).hexdigest() != camera.get("sha256"):
        return ["camera: the frame file is not the inspected file (sha256 differs). Take and inspect a new frame"]
    if not -5 <= written <= PREFLIGHT_MAX_AGE:
        return [f"camera: the frame file is {written:.0f} s old; the limit is {PREFLIGHT_MAX_AGE} s. Take a new frame"]
    try:
        from PIL import Image, ImageStat

        image = Image.open(io.BytesIO(data))
        contrast = ImageStat.Stat(image.convert("L")).stddev[0]
    except Exception as error:
        return [f"camera: the frame is not an image ({error!r}). Take a new frame"]
    if image.width < FRAME_MIN_SIZE[0] or image.height < FRAME_MIN_SIZE[1]:
        return [f"camera: the frame is {image.width}x{image.height}; {FRAME_MIN_SIZE[0]}x{FRAME_MIN_SIZE[1]} is necessary"]
    if contrast < FRAME_MIN_CONTRAST:
        return [f"camera: the frame is blank (contrast {contrast:.1f}). Uncover the lens and take a new frame"]
    return []


def preflight_refusals(record, now):
    """The reasons why this record does not permit a powered run. An empty list permits it.

    Clearance is calculated from the measured distances, not read from a pass or fail word:
      base_separation_cm >= follower_reach_cm + leader_reach_cm + margin_cm   (pan axis to pan axis)
      nearest_person_cm  >= follower_reach_cm + margin_cm   (white pan axis to the nearest part of a person,
                                                             the operator and his hand on the black arm included)
    The two reach values are measured on these arms with the installed tips. A value below the URDF tip-frame
    distance is refused, because the installed tip can only add to that distance.

    These checks are necessary, not sufficient: the program cannot see the table. There is no override.
    """
    if not isinstance(record, dict):
        return ["preflight: the record is not a JSON object"]
    reasons = []

    def section(key):
        if isinstance(record.get(key), dict):
            return record[key]
        reasons.append(f"{key}: missing. A powered run needs this check")

    def age(part, key, where):
        try:
            seconds = (now - datetime.fromisoformat(part.get(key))).total_seconds()
        except (TypeError, ValueError):
            reasons.append(f"{where}: {key} is missing or has no time zone")
            return
        if not -5 <= seconds <= PREFLIGHT_MAX_AGE:
            reasons.append(f"{where}: {key} is {seconds:.0f} s old; the limit is {PREFLIGHT_MAX_AGE} s. Do the check again")

    age(record, "checked_at", "preflight")
    command = section("command")
    if command is not None and (
        set(command) != set(COMMAND_KEYS) or any(type(value) not in (int, float) for value in command.values())
    ):
        reasons.append(f"command: must give {', '.join(COMMAND_KEYS)} of the approved command as numbers")

    clearance = section("clearance")
    if clearance is not None:
        if not isinstance(clearance.get("method"), str) or not clearance["method"].strip():
            reasons.append("clearance: method is missing. Say how each distance was measured")
        keys = ("follower_reach_cm", "leader_reach_cm", "base_separation_cm", "nearest_person_cm", "margin_cm")
        values = [clearance.get(key) for key in keys]
        bad = [key for key, value in zip(keys, values) if type(value) not in (int, float) or not math.isfinite(value) or value <= 0]
        if bad:
            reasons.append(f"clearance: {', '.join(bad)} missing or not a positive number. Measure with a tape")
        else:
            follower_reach, leader_reach, separation, person, margin = values
            for key, value in zip(keys[:2], values):
                if value < REACH_FLOOR_CM:
                    reasons.append(f"clearance: {key} {value} is less than the model value {REACH_FLOOR_CM}. Measure to the installed tip")
            if margin < MIN_MARGIN_CM:
                reasons.append(f"clearance: margin_cm {margin} is less than {MIN_MARGIN_CM}")
            if separation < follower_reach + leader_reach + margin:
                reasons.append(
                    f"clearance: FAILED. The bases are {separation} cm apart; "
                    f"{follower_reach + leader_reach + margin:.1f} cm are necessary. Move the bases apart and measure again"
                )
            if person < follower_reach + margin:
                reasons.append(
                    f"clearance: FAILED. A person is {person} cm from the white pan axis; "
                    f"{follower_reach + margin:.1f} cm are necessary. Move away from the white arm"
                )

    operator = section("operator")
    if operator is not None:
        if operator.get("ready") is not True:
            reasons.append("operator: not ready. The operator must confirm that they are at the black arm")
        age(operator, "confirmed_at", "operator")

    camera = section("camera")
    if camera is not None:
        age(camera, "captured_at", "camera")
        if camera.get("shows_both_arms") is not True:
            reasons.append("camera: the frame does not show both arms. Turn the camera and take a new frame")
        reasons.extend(frame_problems(camera, now))
    return reasons


@dataclass(frozen=True)
class Preflight:
    """A record that passed preflight_refusals at `now`. Only this object arms a run (see Options)."""

    record: dict
    now: datetime

    def __post_init__(self):
        reasons = preflight_refusals(self.record, self.now)
        if reasons:
            raise PreflightRefused("POWERED RUN REFUSED. Nothing was written to a motor.\n- " + "\n- ".join(reasons))

    def require_fresh(self):
        """Called directly before the first motor write: time can pass between the check and that write."""
        times = [self.record["checked_at"], self.record["operator"]["confirmed_at"], self.record["camera"]["captured_at"]]
        expires = min(map(datetime.fromisoformat, times)) + timedelta(seconds=PREFLIGHT_MAX_AGE)
        if datetime.now().astimezone() > expires:
            raise PreflightRefused(f"POWERED RUN REFUSED. Nothing was written to a motor.\n- preflight: expired at {expires:%H:%M:%S}. Do the checks again")


@dataclass(frozen=True)
class Options:
    seconds: float
    speed: int = 80  # ticks per second
    travel: int = 0  # largest distance from the follower start, in ticks; 0 holds the pose
    torque_limit: int = 500  # 0 to 1000, RAM register, restored after the run
    max_error: int = 150  # ticks between the last goal and the read position
    preflight: Preflight | None = None  # None is a dry run. There is no flag that arms a run without the checks

    def __post_init__(self):
        if self.preflight is None:
            return
        if type(self.preflight) is not Preflight:
            raise PreflightRefused("POWERED RUN REFUSED. Nothing was written to a motor.\n- preflight: not a checked record")
        if self.preflight.record["command"] != self.command:
            raise PreflightRefused(
                "POWERED RUN REFUSED. Nothing was written to a motor.\n"
                f"- command: the preflight is for {self.preflight.record['command']}, not for {self.command}"
            )

    @property
    def command(self):
        return {key: getattr(self, key) for key in COMMAND_KEYS}

    @property
    def armed(self):
        return self.preflight is not None


def energized_budget(options):
    """Seconds with torque on, for each phase and in total. The run cannot plan more than this."""
    budget = {
        "enable": ENABLE_BUDGET, "follow": float(options.seconds),
        "return": options.travel / options.speed + RETURN_SETTLE, "release": RELEASE_BUDGET,
    }
    return {**budget, "total": sum(budget.values())}


class FollowerGuard:
    """The only path to a write on the white arm. It checks each write and sends only when armed."""

    def __init__(self, bus, ranges, options, sleep=time.sleep, clock=time.monotonic):
        if options.armed:
            options.preflight.require_fresh()  # an old preflight stops here, before any write
        self.bus, self.armed, self.torque_limit, self.sleep = bus, options.armed, options.torque_limit, sleep
        self.clock, self.energized_at = clock, None  # the time of the first write of the torque-on sequence
        self.window = {name: tuple(ranges[name]) for name in JOINTS}  # the servo position limits
        self.max_step = int(options.speed * PERIOD)
        self.last_goal = self.goal_readback = None
        self.restore, self.write_errors = {}, []
        self.planned, self.sent = Counter(), Counter()

    def _count(self, register, number=1):
        self.planned[register] += number
        if self.armed:
            self.sent[register] += number

    def _write(self, register, name, value, **keywords):
        """One register write on one joint. `sent` counts each write that goes to the bus, also a refused one."""
        self._count(register)
        if not self.armed:
            return
        try:
            self.bus.write(register, name, value, **keywords)
        except Exception as error:
            try:  # the error byte of the next status packet shows which flag the servo has
                flag = self.bus.read_raw("Status", name)[1]
            except Exception:
                flag = None
            self.write_errors.append({"register": register, "joint": name, "value": value, "error": repr(error), "error_flag": flag})
            raise

    def goal(self, targets):
        if set(targets) != set(JOINTS):
            raise ValueError(f"Goal must name the five joints only: {sorted(targets)}")
        for name, value in targets.items():
            low, high = self.window[name]
            if type(value) is not int or not low <= value <= high:
                raise ValueError(f"Goal rejected: {name}={value} is outside {low} to {high}")
            if abs(value - self.last_goal[name]) > self.max_step:
                raise ValueError(f"Goal rejected: {name} step {value - self.last_goal[name]} is more than {self.max_step}")
        self._count("Goal_Position")
        if self.armed:
            self.bus.sync_write("Goal_Position", dict(targets), normalize=False)
        self.last_goal = dict(targets)

    def torque_on(self, present):
        """Limit the torque, set the goal to the present pose, read it back, then enable torque.

        The torque limit goes first: a goal write can switch torque on in some servo firmware.
        For the same reason the energized time starts here, before the first write.
        """
        self.energized_at = self.clock()
        for name in JOINTS:
            if self.armed:
                self.restore[name] = self.bus.read("Torque_Limit", name, normalize=False)
            self._write("Torque_Limit", name, self.torque_limit, normalize=False)
        self.last_goal = dict(present)
        self.goal(dict(present))
        if self.armed:
            readback = self.goal_readback = self.bus.sync_read("Goal_Position", JOINTS, normalize=False, num_retry=2)
            if any(abs(readback[name] - present[name]) > 1 for name in JOINTS):  # a servo changes a goal it refuses
                raise RuntimeError(f"Goal readback {readback} is not the written goal {present}; torque stays off")
        for name in JOINTS:  # the same two writes as FeetechMotorsBus.enable_torque
            self._enable_in_budget()
            self._write("Torque_Enable", name, 1, num_retry=2)
            self._write("Lock", name, 1, num_retry=2)
        self._enable_in_budget()

    def _enable_in_budget(self):
        if self.clock() - self.energized_at > ENABLE_BUDGET:
            raise RuntimeError(f"Torque enable took more than {ENABLE_BUDGET} s; torque goes off")

    def _torque_readback(self, failures, flags, names=JOINTS):
        torque = {}
        for name in names:
            try:
                torque[name], flag = self.bus.read_raw("Torque_Enable", name)
                if flag:
                    flags[name] = flag
            except Exception as error:
                torque[name] = None
                failures.setdefault(name, f"torque readback: {error!r}")
        return torque

    def release(self):
        """Torque off on each joint, also when one joint does not reply. Returns failures and readbacks.

        A servo with an error flag does the write, but LeRobot raises. A servo can also give an OK reply
        and keep torque on (seen on a real white pan). Thus only a readback of 0 shows that torque is off:
        each pass writes, waits and reads, and a joint that still reads on gets the next pass.

        The full torque limit goes back only on a joint that reads torque off directly before that write.
        A joint that reads on, or does not reply, keeps the low limit of this run.

        Time: a lost reply costs about 50 ms for each attempt. A new pass starts only in the first
        RELEASE_PASS_WINDOW seconds, so the release ends in RELEASE_BUDGET also when no joint replies.
        """
        if not self.armed:
            return {"failures": {}, "torque": None, "error_flags": {}, "passes": [],
                    "verified": None, "limit_kept_low": [], "torque_limit": None, "seconds": 0.0}
        failures, flags, passes, pending = {}, {}, [], list(JOINTS)
        started = self.clock()
        for index in range(RELEASE_PASSES):
            if index and self.clock() - started > RELEASE_PASS_WINDOW:
                break
            for register in ("Torque_Enable", "Lock"):
                for name in pending:
                    try:
                        self._write(register, name, 0, num_retry=2)
                    except Exception as error:
                        failures.setdefault(name, f"{register} off: {error!r}")
            self.sleep(RELEASE_SETTLE)
            passes.append(self._torque_readback(failures, flags))
            pending = [name for name in JOINTS if passes[-1][name] != 0]
            if not pending:
                break
        kept = []
        for name, value in self.restore.items():
            if self._torque_readback(failures, flags, [name])[name] != 0:  # on, or no reply: the low limit stays
                kept.append(name)
                continue
            try:
                self._write("Torque_Limit", name, value, normalize=False)
            except Exception as error:
                failures.setdefault(name, f"torque limit restore: {error!r}")
        self.sleep(RELEASE_SETTLE)
        torque = self._torque_readback(failures, flags)  # the last word: read after all writes
        limit = {}
        for name in JOINTS:
            try:
                limit[name] = self.bus.read_raw("Torque_Limit", name)[0]
            except Exception:
                limit[name] = None
        return {
            "failures": failures, "torque": torque, "error_flags": flags, "passes": passes,
            "verified": all(value == 0 for value in torque.values()), "limit_kept_low": kept, "torque_limit": limit,
            "seconds": self.clock() - started,
        }


def release_verdict(armed, report):
    """(released, text). Released only when the readback after the last write shows torque 0 on the five joints.

    An OK reply to a write and an arm that does not move are not evidence of release.
    """
    if not armed:
        return True, "Dry run: nothing was written to a motor."
    release = report.get("release")
    if release is None and not report.get("motor_writes"):
        return True, "Stopped before any motor write."
    torque = (release or {}).get("torque") or {}
    unconfirmed = [name for name in JOINTS if torque.get(name) != 0]
    if unconfirmed:
        return False, (
            f"RELEASE UNVERIFIED on {unconfirmed}: torque reads on, or there is no reply. "
            f"Low torque limit kept on {(release or {}).get('limit_kept_low')}. "
            "DO THIS NOW: hold the white arm with one hand. Pull the DC barrel plug out of the white controller "
            "board. Leave the USB cable connected. Do not push the arm against its motors."
        )
    text = "Torque reads off on white joints 1 to 5."
    used, budget = report.get("energized_seconds"), (report.get("budget_seconds") or {}).get("total")
    if used is not None and budget is not None:
        text += f" Energized {used:.1f} s of the {budget:.1f} s budget."
        if used > budget:
            text += " ENERGIZED TIME WAS MORE THAN THE BUDGET."
    if release["limit_kept_low"]:
        text += f" Low torque limit still on {release['limit_kept_low']}."
    if release["failures"]:
        text += f" Release writes gave errors: {release['failures']}"
    return True, text


def check_ready(bus, key, expected):
    if any(bus.ping(name) != 777 for name in JOINTS):
        raise RuntimeError(f"{key}: motor replies missing")
    if any(bus.read("Torque_Enable", name) for name in JOINTS):
        raise RuntimeError(f"{key}: torque must be off at the start")
    actual = bus.read_calibration()
    if any(asdict(actual[name]) != expected[name] for name in JOINTS):
        raise RuntimeError(f"{key}: saved five-joint calibration does not match the motors")


def check_health(follower):
    status = follower.sync_read("Status", JOINTS, normalize=False, num_retry=2)
    temperature = follower.sync_read("Present_Temperature", JOINTS, normalize=False, num_retry=2)
    if any(status.values()):
        raise RuntimeError(f"Follower fault status {status}")
    if max(temperature.values()) >= MAX_TEMPERATURE:
        raise RuntimeError(f"Follower temperature {temperature} is {MAX_TEMPERATURE} C or more")


def run(leader, follower, expected, options, report, clock=time.monotonic, sleep=time.sleep, notify=lambda phase: None):
    """Checks, then follow, then return to the start pose, then torque off. Fills `report`."""
    check_ready(leader, "leader", expected["leader"])
    check_ready(follower, "follower", expected["follower"])
    if any(follower.read("Operating_Mode", name) != 0 for name in JOINTS):
        raise RuntimeError("follower: a motor is not in position mode")
    check_health(follower)
    flags = {name: follower.read_raw("Status", name)[1] for name in JOINTS}
    if any(flags.values()):
        raise RuntimeError(f"Follower status packets have an error flag {flags}")
    ranges = {name: (entry["range_min"], entry["range_max"]) for name, entry in expected["follower"].items()}
    limits = {name: (low + MARGIN, high - MARGIN) for name, (low, high) in ranges.items()}
    leader_start = leader.sync_read("Present_Position", JOINTS, normalize=False, num_retry=2)
    follower_present = follower.sync_read("Present_Position", JOINTS, normalize=False, num_retry=2)
    for name, (low, high) in ranges.items():
        if not low - RANGE_SLACK <= follower_present[name] <= high + RANGE_SLACK:
            raise RuntimeError(f"Follower {name} is outside its recorded range; calibration does not fit this pose")
    # The servo changes a goal that is outside its position limits. A joint that rests on its stop can read
    # a few ticks outside them. Thus the start pose is the present pose moved into the limits.
    follower_start = {name: max(ranges[name][0], min(ranges[name][1], follower_present[name])) for name in JOINTS}
    report.update(follower_present=follower_present, follower_start=follower_start)

    guard = FollowerGuard(follower, ranges, options, sleep, clock)
    rows = report.setdefault("rows", [])
    budget = energized_budget(options)
    report.update(planned_writes=guard.planned, motor_writes=guard.sent, result="aborted", budget_seconds=budget)
    try:
        guard.torque_on(follower_start)
        notify("follow")
        previous, phase, bad, cycle, still, follower_last = follower_start, "follow", 0, 0, 0, follower_present
        start = last = clock()
        # Five cycles before the end of the return budget: the last cycle and the last sleep stay inside it.
        return_deadline = start + options.seconds + budget["return"] - 5 * PERIOD
        while True:
            tick = clock()
            leader_now = leader.sync_read("Present_Position", JOINTS, normalize=False, num_retry=2)
            follower_now = follower.sync_read("Present_Position", JOINTS, normalize=False, num_retry=2)
            now = clock()
            if now - last > 0.25:
                raise RuntimeError("Position reads delayed more than 250 ms")
            error = max(abs(guard.last_goal[name] - follower_now[name]) for name in JOINTS)
            bad = bad + 1 if options.armed and error > options.max_error else 0
            if bad >= BAD_SAMPLES:
                raise RuntimeError(f"Follower does not follow: {error} ticks from the goal")
            if cycle % 10 == 9:
                check_health(follower)
            if phase == "follow" and now - start >= options.seconds:
                phase = "return"
                notify("return")
            if phase == "follow":
                targets = next_targets(
                    leader_start, leader_now, follower_start, previous, limits, now - last,
                    options.speed, options.travel,
                )
            else:  # leader at its start and zero travel: step back to the follower start
                targets = next_targets(
                    leader_start, leader_start, follower_start, previous, limits, now - last, options.speed, 0
                )
            guard.goal(targets)
            rows.append({
                "elapsed_seconds": now - start, "phase": phase,
                "leader_raw": leader_now, "follower_raw": follower_now, "goal_raw": targets,
            })
            if phase == "return":
                home_error = max(abs(follower_now[name] - follower_start[name]) for name in JOINTS)
                moved = max(abs(follower_now[name] - follower_last[name]) for name in JOINTS)
                # `previous` is the goal of the cycle before: the arm had one full cycle to reach it.
                at_start_goal = previous == follower_start
                still = still + 1 if at_start_goal and moved <= STILL_TICKS else 0
                if at_start_goal and (not options.armed or home_error <= HOME_TOLERANCE):
                    report["result"] = "complete"
                elif still >= STILL_CYCLES:  # a low torque limit stops the arm some ticks from the start
                    report["result"] = "settled short of start"
                elif now > return_deadline:
                    report["result"] = "return to start timed out"
                if report["result"] != "aborted":
                    report["home_error"] = home_error
                    sleep(PERIOD)  # the servos get the last goal before the torque-off writes
                    break
            previous, last, cycle, follower_last = targets, now, cycle + 1, follower_now
            sleep(max(0, PERIOD - (clock() - tick)))
    finally:
        report.update(goal_readback=guard.goal_readback, write_errors=guard.write_errors)
        report["release"] = guard.release()
        if guard.energized_at is not None:
            report["energized_seconds"] = clock() - guard.energized_at


# ---------------------------------------------------------------- offline self-test


@dataclass
class FakeCalibration:
    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int


class Sim:
    """Simulated time. A sleep moves the fake follower."""

    def __init__(self):
        self.t, self.follower = 0.0, None

    def clock(self):
        return self.t

    def sleep(self, seconds):
        self.spend(seconds + 0.001)  # loop overhead, as on the real arms

    def spend(self, seconds):
        self.t += seconds
        assert self.t < 600, "the simulated run did not end"
        self.follower.advance(seconds)

    def lost(self, attempts):
        """A lost reply costs one packet timeout for each attempt: about 50 ms (lerobot feetech.py)."""
        self.spend(0.05 * attempts)


class FakeBus:
    def __init__(self, sim, positions, script=None):
        self.sim, self.pos, self.script = sim, dict(positions), script
        self.goal = {name: value + 500 for name, value in positions.items()}  # stale goal register
        self.torque = dict.fromkeys(JOINTS, 0)
        self.registers = {"Operating_Mode": 0, "Status": 0, "Present_Temperature": 35, "Torque_Limit": 1000}
        self.torque_limit = dict.fromkeys(JOINTS, 1000)
        self.calibration = {
            name: FakeCalibration(index + 1, 0, 0, 500, 3500) for index, name in enumerate(JOINTS)
        }
        self.log, self.stuck, self.fail_disable, self.fail_after = [], set(), set(), None
        self.flag, self.flag_on_torque, self.goal_shift, self.lose_off = {}, set(), {}, {}
        self.silent_from = {}  # joint name to the time from which it gives no reply
        self.short = {}  # joint name to the ticks that it stops short of its goal (low torque limit)
        self.wobble, self.flip = {}, 1  # joint name to (offset, amplitude): moves all the time, never at the goal
        self.slow_write = 0.0  # seconds that each register write takes

    def advance(self, seconds):
        self.flip = -self.flip
        for name in JOINTS:
            if self.torque[name] and name in self.wobble:
                offset, amplitude = self.wobble[name]
                self.pos[name] = self.goal[name] + offset + amplitude * self.flip
            elif self.torque[name] and name not in self.stuck:
                gap = self.goal[name] - self.pos[name]
                dead = self.short.get(name, 0)
                if abs(gap) > dead:
                    gap -= dead if gap > 0 else -dead
                    self.pos[name] += max(-int(600 * seconds), min(int(600 * seconds), gap))

    def _silent(self, name):
        return self.sim.t >= self.silent_from.get(name, float("inf"))

    def ping(self, name):
        return 777 if name in self.pos else None

    def read(self, register, name, normalize=True, attempts=1):
        if self._silent(name):
            self.sim.lost(attempts)
            raise ConnectionError("no status packet")
        if register == "Torque_Enable":
            return self.torque[name]
        if register == "Torque_Limit":
            return self.torque_limit[name]
        return self.registers[register]

    def read_raw(self, register, name):
        return self.read(register, name, attempts=3), self.flag.get(name, 0)  # the real read_raw tries 3 times

    def read_calibration(self):
        return self.calibration

    def sync_read(self, register, names, normalize=True, num_retry=0):
        if (self.fail_after is not None and self.sim.t > self.fail_after) or any(self._silent(name) for name in names):
            self.sim.lost(1 + num_retry)
            raise ConnectionError("no status packet")
        if register == "Present_Position":
            source = self.script(self.sim.t) if self.script else self.pos
        elif register == "Goal_Position":
            source = self.goal
        else:
            source = dict.fromkeys(JOINTS, self.registers[register])
        return {name: source[name] for name in names}

    def sync_write(self, register, values, normalize=True):
        assert register == "Goal_Position" and not normalize, register
        self.log.append((register, dict(values)))
        for name, value in values.items():
            # Seen on the white elbow: a goal outside the position limits is moved to the limit.
            # Hypothesis for the abort seen on a real arm: the servo then sets an error flag.
            self.goal[name] = max(self.calibration[name].range_min, min(self.calibration[name].range_max, value))
            self.goal[name] += self.goal_shift.get(name, 0)
            if self.goal[name] != value and name not in self.goal_shift:
                self.flag[name] = UNNAMED_FLAG

    def write(self, register, name, value, normalize=True, num_retry=0):
        if (register == "Torque_Enable" and value == 0 and name in self.fail_disable) or self._silent(name):
            self.sim.lost(1 + num_retry)
            raise ConnectionError("no status packet")
        if self.slow_write:
            self.sim.spend(self.slow_write)
        self.log.append((register, name, value))
        if register == "Torque_Limit":
            self.torque_limit[name] = value
        elif register == "Torque_Enable":
            if value == 0 and self.lose_off.get(name, 0) > 0:
                self.lose_off[name] -= 1  # seen on a real white pan servo: an OK reply, but torque stays on
            else:
                self.torque[name] = value
            if value and name in self.flag_on_torque:
                self.flag[name] = UNNAMED_FLAG
        else:
            assert register == "Lock", register
        if self.flag.get(name):  # the servo does the write, but LeRobot raises on the flag
            raise RuntimeError(f"Failed to write '{register}' on {name}")


FIXTURE = {}


def fixture_record(command, /, now=None, **sections):
    """A preflight record that passes, for the fake arms only. `sections` replaces or changes sections."""
    if not FIXTURE:
        from PIL import Image

        FIXTURE["folder"] = tempfile.TemporaryDirectory(prefix="so101-preflight-")
        FIXTURE["frame"] = Path(FIXTURE["folder"].name) / "frame.jpg"
        Image.effect_noise((640, 360), 64).save(FIXTURE["frame"])
        FIXTURE["sha256"] = hashlib.sha256(FIXTURE["frame"].read_bytes()).hexdigest()
    now = now or datetime.now().astimezone()
    os.utime(FIXTURE["frame"], (now.timestamp(), now.timestamp()))
    stamp = now.isoformat()
    record = {
        "checked_at": stamp, "command": dict(command),
        "clearance": {
            "follower_reach_cm": 55.0, "leader_reach_cm": 55.0, "base_separation_cm": 135.0,
            "nearest_person_cm": 80.0, "margin_cm": 20.0, "method": "fixture for the fake arms",
        },
        "operator": {"ready": True, "confirmed_at": stamp},
        "camera": {"frame": str(FIXTURE["frame"]), "sha256": FIXTURE["sha256"], "captured_at": stamp, "shows_both_arms": True},
    }
    for key, value in sections.items():
        record[key] = {**record[key], **value} if isinstance(value, dict) else value
    return {key: value for key, value in record.items() if value is not None}


def scene(leader_motion=None, armed=False, **options):
    """Fake leader and follower. leader_motion maps joint to (ticks per second, total ticks)."""
    options = Options(**options)
    if armed:
        now = datetime.now().astimezone()
        options = replace(options, preflight=Preflight(fixture_record(options.command, now), now))
    sim = Sim()
    leader_start = dict(zip(JOINTS, (1000, 1500, 2000, 2500, 2048)))
    follower_start = dict(zip(JOINTS, (2000, 1600, 1800, 2200, 2048)))

    def script(t):
        now = dict(leader_start, gripper=1234)
        for name, (rate, total) in (leader_motion or {}).items():
            now[name] += int(max(-abs(total), min(abs(total), rate * t)))
        return now

    leader, follower = FakeBus(sim, leader_start, script), FakeBus(sim, follower_start)
    sim.follower = follower
    expected = {
        key: {name: asdict(value) for name, value in bus.calibration.items()}
        for key, bus in (("leader", leader), ("follower", follower))
    }
    return sim, leader, follower, follower_start, expected, options


def go(sim, leader, follower, expected, options):
    report = {"events": []}
    try:
        run(leader, follower, expected, options, report, sim.clock, sim.sleep, report["events"].append)
    except Exception as error:
        report["error"] = error
    return report


def self_test():
    off = dict.fromkeys(JOINTS, 0)

    # Hold only: the stale goal register must not move the arm, and a moving leader must not either.
    sim, leader, follower, start, expected, options = scene({"shoulder_pan": (100, 300)}, seconds=3, armed=True)
    report = go(sim, leader, follower, expected, options)
    assert "error" not in report and report["result"] == "complete", report
    assert all(row["follower_raw"] == start for row in report["rows"]) and follower.pos == start
    assert report["events"] == ["follow", "return"] and report["release"]["verified"] is True
    assert release_verdict(True, report)[0] and report["release"]["torque_limit"] == dict.fromkeys(JOINTS, 1000)
    first_torque = next(i for i, entry in enumerate(follower.log) if entry[0] == "Torque_Enable")
    assert follower.log[:first_torque] == [("Torque_Limit", n, 500) for n in JOINTS] + [("Goal_Position", start)]
    assert follower.torque == off and follower.torque_limit == dict.fromkeys(JOINTS, 1000)
    assert {entry[0] for entry in follower.log} == {"Goal_Position", "Torque_Limit", "Torque_Enable", "Lock"}
    assert leader.log == [] and "gripper" not in report["rows"][-1]["goal_raw"]

    # Follow: asymmetric motion, leader faster than the cap, then return to the start and release.
    sim, leader, follower, start, expected, options = scene(
        {"shoulder_pan": (100, 200), "shoulder_lift": (-100, 150)}, seconds=6, travel=400, armed=True
    )
    report = go(sim, leader, follower, expected, options)
    assert "error" not in report and report["result"] == "complete", report
    rows = report["rows"]
    assert max(row["follower_raw"]["shoulder_pan"] for row in rows) == 2200
    assert min(row["follower_raw"]["shoulder_lift"] for row in rows) == 1450
    assert all(row["follower_raw"][n] == start[n] for row in rows for n in JOINTS[2:])
    steps = [abs(b["goal_raw"][n] - a["goal_raw"][n]) for a, b in zip(rows, rows[1:]) for n in JOINTS]
    assert max(steps) == 8, max(steps)
    assert follower.pos == start and follower.torque == off and rows[-1]["phase"] == "return"

    # Travel limit: the same leader motion with travel 60 stops at 60 ticks.
    sim, leader, follower, start, expected, options = scene({"shoulder_pan": (100, 200)}, seconds=4, travel=60, armed=True)
    report = go(sim, leader, follower, expected, options)
    assert max(row["follower_raw"]["shoulder_pan"] for row in report["rows"]) == 2060

    # Blocked joint: torque goes off soon, and the goal does not run away.
    sim, leader, follower, start, expected, options = scene({"elbow_flex": (-100, 400)}, seconds=10, travel=1000, armed=True)
    follower.stuck = {"elbow_flex"}
    report = go(sim, leader, follower, expected, options)
    assert "does not follow" in str(report["error"]), report.get("error")
    assert follower.torque == off and 150 < 1800 - report["rows"][-1]["goal_raw"]["elbow_flex"] <= 190

    # Lost leader replies during the run: torque goes off.
    sim, leader, follower, start, expected, options = scene({"shoulder_pan": (50, 100)}, seconds=5, travel=200, armed=True)
    leader.fail_after = 1.0
    report = go(sim, leader, follower, expected, options)
    assert isinstance(report["error"], ConnectionError) and follower.torque == off
    assert report["release"]["torque"] == off

    # One joint does not reply at release: the other four still go off, and the failure is reported.
    sim, leader, follower, start, expected, options = scene(seconds=1, armed=True)
    follower.fail_disable = {"shoulder_pan"}
    report = go(sim, leader, follower, expected, options)
    assert set(report["release"]["failures"]) == {"shoulder_pan"}
    assert follower.torque == {**off, "shoulder_pan": 1}
    assert report["release"]["torque"] == {**off, "shoulder_pan": 1} and len(report["release"]["passes"]) == RELEASE_PASSES
    # The joint that reads on keeps the low limit of the run (500 here); the confirmed joints get 1000 back.
    assert follower.torque_limit == {**dict.fromkeys(JOINTS, 1000), "shoulder_pan": 500}, follower.torque_limit
    assert report["release"]["limit_kept_low"] == ["shoulder_pan"] and report["release"]["verified"] is False
    assert ("Torque_Limit", "shoulder_pan", 1000) not in follower.log
    released, text = release_verdict(True, report)
    assert not released and "shoulder_pan" in text and "DC barrel plug" in text

    # A torque-off write gets an OK reply, but torque stays on (seen on a real white pan): the readback sees it,
    # and the next pass turns it off.
    sim, leader, follower, start, expected, options = scene(seconds=1, armed=True)
    follower.lose_off = {"shoulder_pan": 1}
    report = go(sim, leader, follower, expected, options)
    assert "error" not in report and report["result"] == "complete" and not report["release"]["failures"], report
    assert report["release"]["passes"] == [{**off, "shoulder_pan": 1}, off], report["release"]
    assert follower.torque == off and report["release"]["torque"] == off
    assert follower.torque_limit == dict.fromkeys(JOINTS, 1000)

    # A joint that never turns off, with OK replies: the report must not say "off".
    sim, leader, follower, start, expected, options = scene(seconds=1, armed=True)
    follower.lose_off = {"wrist_flex": 99}
    report = go(sim, leader, follower, expected, options)
    assert report["release"]["torque"] == {**off, "wrist_flex": 1} and not report["release"]["failures"]
    assert len(report["release"]["passes"]) == RELEASE_PASSES and follower.torque == {**off, "wrist_flex": 1}
    # OK replies to the off writes are not evidence: the limit stays low, and the release is unverified.
    assert follower.torque_limit == {**dict.fromkeys(JOINTS, 1000), "wrist_flex": 500}, follower.torque_limit
    assert ("Torque_Limit", "wrist_flex", 1000) not in follower.log and not release_verdict(True, report)[0]
    assert report["release"]["torque_limit"]["wrist_flex"] == 500

    # A joint stops replying during the run: its torque state is unknown, so its limit stays low.
    sim, leader, follower, start, expected, options = scene(seconds=5, armed=True)
    follower.silent_from = {"elbow_flex": 1.0}
    report = go(sim, leader, follower, expected, options)
    assert isinstance(report["error"], ConnectionError), report
    assert report["release"]["torque"] == {**off, "elbow_flex": None} and report["release"]["verified"] is False
    assert follower.torque_limit == {**dict.fromkeys(JOINTS, 1000), "elbow_flex": 500}, follower.torque_limit
    assert report["release"]["limit_kept_low"] == ["elbow_flex"] and report["release"]["torque_limit"]["elbow_flex"] is None
    released, text = release_verdict(True, report)
    assert not released and "elbow_flex" in text and "DC barrel plug" in text

    # The verdict: no release record after a motor write is unverified; no write at all is not a hazard.
    assert not release_verdict(True, {"motor_writes": {"Torque_Limit": 1}})[0]
    assert release_verdict(True, {})[0] and release_verdict(False, {"release": {"torque": None}})[0]

    # Rest pose two ticks outside the servo limit (seen on a real white elbow): the hold goal moves into the limit.
    sim, leader, follower, start, expected, options = scene(seconds=2, armed=True)
    follower.pos["elbow_flex"] = 3502
    report = go(sim, leader, follower, expected, options)
    assert "error" not in report and report["result"] == "complete", report
    goals = {entry[1]["elbow_flex"] for entry in follower.log if entry[0] == "Goal_Position"}
    assert goals == {3500} and not follower.flag and follower.torque == off, (goals, follower.flag)
    assert report["follower_present"]["elbow_flex"] == 3502 and report["goal_readback"]["elbow_flex"] == 3500

    # A servo sets an error flag at torque enable: all joints still go off, and the raw readback shows it.
    sim, leader, follower, start, expected, options = scene(seconds=1, armed=True)
    follower.flag_on_torque = {"elbow_flex"}
    report = go(sim, leader, follower, expected, options)
    assert isinstance(report["error"], RuntimeError) and report["rows"] == [], report
    assert report["events"] == [], report["events"]  # no "move now" cue before torque is on for all joints
    assert report["release"]["torque"] == off and report["release"]["error_flags"] == {"elbow_flex": UNNAMED_FLAG}
    assert follower.torque == off and follower.torque_limit == dict.fromkeys(JOINTS, 1000)
    assert ("Lock", "elbow_flex", 0) in follower.log and ("Lock", "shoulder_pan", 0) in follower.log
    # Sent: enable on pan, lift, elbow and lock on pan, lift; then off and unlock on five joints.
    assert report["motor_writes"]["Torque_Enable"] == 8 and report["motor_writes"]["Lock"] == 7, report["motor_writes"]
    assert report["write_errors"][0]["joint"] == "elbow_flex" and report["write_errors"][0]["error_flag"] == UNNAMED_FLAG

    # The servo stores a goal that is not the written goal: no torque enable.
    sim, leader, follower, start, expected, options = scene(seconds=1, armed=True)
    follower.goal_shift = {"wrist_flex": 2}
    report = go(sim, leader, follower, expected, options)
    assert "Goal readback" in str(report["error"]) and report["rows"] == [] and report["events"] == [], report
    assert not any(entry[0] == "Torque_Enable" and entry[2] == 1 for entry in follower.log), follower.log
    assert follower.torque == off and follower.torque_limit == dict.fromkeys(JOINTS, 1000)

    # ---------------- energized-time budget
    def return_seconds(report, options):  # from the end of the follow time to the first torque-off write
        return report["rows"][-1]["elapsed_seconds"] + PERIOD - options.seconds

    full = dict.fromkeys(JOINTS, 1000)

    # Low torque: joints stop 20 ticks short of the start (seen on a real white arm: torque stayed on 54 s more).
    # The return ends when the arm is static. Release is verified and the full limit comes back.
    sim, leader, follower, start, expected, options = scene({"shoulder_pan": (100, 300)}, seconds=3, travel=100, armed=True)
    follower.short = {"shoulder_pan": 20, "wrist_flex": 21}
    report = go(sim, leader, follower, expected, options)
    budget = energized_budget(options)
    assert budget == {"enable": 3.0, "follow": 3.0, "return": 4.25, "release": 8.0, "total": 18.25}, budget
    assert "error" not in report and report["result"] == "settled short of start", report
    assert report["home_error"] == 20 and return_seconds(report, options) < 100 / 80 + 1.2, return_seconds(report, options)
    assert report["energized_seconds"] <= budget["total"] and report["budget_seconds"] == budget
    assert report["release"]["verified"] is True and follower.torque == off and follower.torque_limit == full
    assert release_verdict(True, report)[0] and "of the 18.2 s budget" in release_verdict(True, report)[1]

    # Unreachable home: a joint moves all the time and never gets to the start. The return ends at its limit.
    sim, leader, follower, start, expected, options = scene({"shoulder_pan": (100, 300)}, seconds=3, travel=100, armed=True)
    follower.wobble = {"shoulder_lift": (40, 4)}
    report = go(sim, leader, follower, expected, options)
    assert "error" not in report and report["result"] == "return to start timed out", report
    assert budget["return"] - 1 < return_seconds(report, options) <= budget["return"], return_seconds(report, options)
    assert report["energized_seconds"] <= budget["total"], report["energized_seconds"]
    assert report["release"]["verified"] is True and follower.torque == off and follower.torque_limit == full

    # The same stalled return, and the pan keeps torque on with OK replies: the attempts end in the budget,
    # the pan keeps the low limit of the run, and the verdict tells the user to pull the DC plug.
    sim, leader, follower, start, expected, options = scene(
        {"shoulder_pan": (100, 300)}, seconds=3, travel=100, torque_limit=300, armed=True
    )
    follower.wobble, follower.lose_off = {"shoulder_lift": (40, 4)}, {"shoulder_pan": 99}
    report = go(sim, leader, follower, expected, options)
    assert report["result"] == "return to start timed out" and report["energized_seconds"] <= budget["total"]
    assert report["release"]["verified"] is False and report["release"]["limit_kept_low"] == ["shoulder_pan"]
    assert follower.torque_limit == {**full, "shoulder_pan": 300} and follower.torque == {**off, "shoulder_pan": 1}
    released, text = release_verdict(True, report)
    assert not released and "shoulder_pan" in text and "DC barrel plug" in text

    # No reply from any set of joints: the release attempts end in RELEASE_BUDGET. This limits the software
    # only. A joint that does not reply keeps its torque and the low limit until the DC plug is pulled.
    slowest = 0.0
    for size in range(1, len(JOINTS) + 1):
        for silent in combinations(JOINTS, size):
            sim, leader, follower, start, expected, options = scene(seconds=3, travel=100, armed=True)
            follower.silent_from = dict.fromkeys(silent, 1.5)
            report = go(sim, leader, follower, expected, options)
            assert isinstance(report["error"], ConnectionError), (silent, report["error"])
            assert report["release"]["seconds"] <= RELEASE_BUDGET, (silent, report["release"]["seconds"])
            assert report["energized_seconds"] <= budget["total"], (silent, report["energized_seconds"])
            assert report["release"]["limit_kept_low"] == list(silent) and report["release"]["verified"] is False
            assert follower.torque_limit == {**full, **dict.fromkeys(silent, 500)}, (silent, follower.torque_limit)
            assert not release_verdict(True, report)[0]
            slowest = max(slowest, report["release"]["seconds"])
    self_test.slowest_release = slowest

    # Slow replies at torque enable: the enable budget stops the run before the "move now" cue.
    sim, leader, follower, start, expected, options = scene(seconds=1, armed=True)
    follower.slow_write = 0.4
    report = go(sim, leader, follower, expected, options)
    assert "Torque enable took more" in str(report["error"]) and report["events"] == [] and report["rows"] == [], report
    assert report["motor_writes"]["Torque_Enable"] == 7, report["motor_writes"]  # 2 on, then 5 off
    assert report["release"]["verified"] is True and follower.torque == off and follower.torque_limit == full
    assert report["energized_seconds"] <= energized_budget(options)["total"], report["energized_seconds"]

    # The verdict shows energized time over the budget.
    over = {"release": {"torque": off, "limit_kept_low": [], "failures": {}}, "energized_seconds": 20.0,
            "budget_seconds": {"total": 18.25}}
    assert "MORE THAN THE BUDGET" in release_verdict(True, over)[1]
    assert "MORE THAN THE BUDGET" not in release_verdict(True, {**over, "energized_seconds": 18.0})[1]

    # Refusals: no write at all.
    def torque_on(bus):
        bus.torque["elbow_flex"] = 1

    def wrong_calibration(bus):
        bus.calibration["wrist_flex"].homing_offset = 5

    def wrong_mode(bus):
        bus.registers["Operating_Mode"] = 1

    def fault(bus):
        bus.registers["Status"] = 1

    def out_of_range(bus):
        bus.pos["shoulder_lift"] = 400

    def flagged(bus):
        bus.flag["wrist_roll"] = UNNAMED_FLAG

    for change, text in (
        (torque_on, "torque must be off"), (wrong_calibration, "calibration"), (wrong_mode, "position mode"),
        (fault, "fault status"), (out_of_range, "outside its recorded range"), (flagged, "error flag"),
    ):
        sim, leader, follower, start, expected, options = scene(seconds=1, travel=100, armed=True)
        change(follower)
        report = go(sim, leader, follower, expected, options)
        assert text in str(report["error"]) and follower.log == [], (text, report.get("error"), follower.log)
        assert report["events"] == [] and release_verdict(True, report)[0]

    # Dry run: the goals move, but no write reaches the follower.
    sim, leader, follower, start, expected, options = scene({"shoulder_pan": (100, 200)}, seconds=4, travel=400)
    report = go(sim, leader, follower, expected, options)
    assert report["result"] == "complete" and follower.log == [] and not report["motor_writes"]
    assert max(row["goal_raw"]["shoulder_pan"] for row in report["rows"]) == 2200
    assert report["planned_writes"]["Torque_Enable"] == 5 and report["release"]["torque"] is None

    # The guard itself.
    guard = FollowerGuard(follower, {name: (500, 3500) for name in JOINTS}, scene(seconds=1, armed=True)[-1])
    guard.last_goal = dict(start)
    for bad in (
        {**start, "gripper": 2000}, {name: start[name] for name in JOINTS[:4]},
        {**start, "shoulder_pan": 2009}, {**start, "shoulder_pan": 2004.0},
    ):
        try:
            guard.goal(bad)
        except ValueError:
            continue
        raise AssertionError(f"guard accepted {bad}")
    guard.last_goal = {**start, "shoulder_pan": 3498}
    try:
        guard.goal({**start, "shoulder_pan": 3503})
    except ValueError:
        pass
    else:
        raise AssertionError("guard accepted a goal outside the range")
    assert follower.log == []

    preflight_test()


def preflight_test():
    """Incident (real): the approved command started with torque while the clearance check had failed.

    Each case gives the command line of that run to main(). The fake arms stand in for the serial ports.
    A refusal must end before the arms are opened, with no write on the white arm.
    """
    command = {"seconds": 300, "speed": 400, "travel": 1200, "torque_limit": 500}
    approved = ["--enable-torque", "--seconds", "300", "--speed", "400", "--travel", "1200", "--torque-limit", "500"]
    sim, leader, follower, _, expected, _ = scene(seconds=1)
    reports = []

    def fake_arms(options):
        reports.append(go(sim, leader, follower, expected, options))

    def attempt(record, argv=approved):
        """The exit code and the text of the refusal."""
        with tempfile.TemporaryDirectory() as folder, contextlib.redirect_stderr(io.StringIO()) as text:
            if record is not None:
                path = Path(folder) / "preflight.json"
                path.write_text(record if isinstance(record, str) else json.dumps(record))
                argv = [*argv, "--preflight", str(path)]
            try:
                main(argv, fake_arms)
            except SystemExit as stop:
                return stop.code, text.getvalue()
        return 0, text.getvalue()

    now = datetime.now().astimezone()
    old = (now - timedelta(seconds=PREFLIGHT_MAX_AGE + 60)).isoformat()
    fixture_record(command)  # makes the fixture folder
    folder = Path(FIXTURE["folder"].name)
    from PIL import Image

    Image.new("RGB", (1280, 720)).save(folder / "black.jpg")
    (folder / "text.jpg").write_text("not an image")

    def other_frame(name):
        return {"frame": str(folder / name), "sha256": hashlib.sha256((folder / name).read_bytes()).hexdigest()}

    refusals = {
        "the incident command with no record": (None, "--enable-torque needs --preflight"),
        # The incident: bases about 55 cm apart, a hand next to the white base, operator ready, camera frame good.
        "failed clearance (the incident)": (
            {"clearance": {"follower_reach_cm": 47.8, "leader_reach_cm": 47.8, "base_separation_cm": 55, "nearest_person_cm": 10}},
            "clearance: FAILED. The bases are 55 cm apart",
        ),
        "a person inside the reach, bases far apart": ({"clearance": {"nearest_person_cm": 60}}, "clearance: FAILED. A person is 60 cm"),
        "clearance not measured": ({"clearance": None}, "clearance: missing"),
        "clearance given as a word": ({"clearance": "pass"}, "clearance: missing"),
        "one distance not measured": ({"clearance": {"base_separation_cm": None}}, "base_separation_cm missing"),
        "reach below the model value": ({"clearance": {"follower_reach_cm": 30}}, "less than the model value"),
        "margin too small": ({"clearance": {"margin_cm": 5}}, "margin_cm 5 is less than"),
        "no measurement method": ({"clearance": {"method": " "}}, "method is missing"),
        "readiness missing": ({"operator": None}, "operator: missing"),
        "operator not ready": ({"operator": {"ready": False}}, "operator: not ready"),
        "readiness as a word": ({"operator": {"ready": "yes"}}, "operator: not ready"),
        "old readiness": ({"operator": {"confirmed_at": old}}, "operator: confirmed_at is"),
        "no camera evidence": ({"camera": None}, "camera: missing"),
        "frame file gone": ({"camera": {"frame": str(folder / "none.jpg")}}, "frame file is not readable"),
        "frame is not the inspected file": ({"camera": {"sha256": "0" * 64}}, "sha256 differs"),
        "blank frame": ({"camera": other_frame("black.jpg")}, "the frame is blank"),
        "frame is not an image": ({"camera": other_frame("text.jpg")}, "not an image"),
        "arms out of the view": ({"camera": {"shows_both_arms": False}}, "does not show both arms"),
        "old frame": ({"camera": {"captured_at": old}}, "camera: captured_at is"),
        "old record": ({"checked_at": old}, "preflight: checked_at is"),
        "record for a smaller command": ({"command": {**command, "travel": 300}}, "the preflight is for"),
    }
    for label, (changes, reason) in refusals.items():
        record = None if changes is None else fixture_record(command, **changes)
        code, text = attempt(record)
        assert code == 3 and reason in text, (label, code, text)
        assert "Nothing was written to a motor" in text or changes is None, (label, text)
        assert reports == [] and follower.log == [], (label, follower.log)
    code, text = attempt("{not json")
    assert code == 3 and "--enable-torque needs --preflight" in text and reports == [] and follower.log == [], (code, text)

    # No other way arms a run: the old `armed` flag is gone, and a record alone is not a checked record.
    for attempt_options in (lambda: Options(seconds=300, armed=True), lambda: Options(seconds=300, preflight=fixture_record(command))):
        try:
            attempt_options()
        except (TypeError, PreflightRefused):
            continue
        raise AssertionError("a run was armed without a checked preflight")

    # A preflight that was good at the check, but is old at the first write: refused, no write.
    past = now - timedelta(seconds=PREFLIGHT_MAX_AGE + 120)
    stale = Preflight(fixture_record(command, past), past + timedelta(seconds=60))
    report = go(sim, leader, follower, expected, Options(**command, preflight=stale))
    assert isinstance(report["error"], PreflightRefused) and follower.log == [], (report["error"], follower.log)
    assert "motor_writes" not in report and release_verdict(True, report)[0]

    # Control: the same command with a record that passes does reach the motors. The refusals above are thus real.
    code, text = attempt(fixture_record(command))
    assert code == 0 and len(reports) == 1 and "error" not in reports[0], (code, text, reports)
    assert reports[0]["motor_writes"]["Torque_Enable"] >= 10 and follower.log, reports[0]["motor_writes"]
    # A dry run needs no record, and a record does not arm a dry run.
    code, text = attempt(None, approved[1:])
    assert code == 0 and not reports[1]["motor_writes"], (code, reports[1]["motor_writes"])


# ---------------------------------------------------------------- hardware


def hardware(options):
    safe = runpy.run_path(str(ROOT / "so101-calibrate-safe.py"))

    from lerobot.motors.motors_bus import get_address

    class Bus(safe["FeetechMotorsBus"]):
        def read_raw(self, register, name):
            """Value and error byte of the status packet. An error flag does not raise here."""
            motor = self.motors[name]
            address, length = get_address(self.model_ctrl_table, motor.model, register)
            value, comm, error = self._read(address, length, motor.id, num_retry=2, raise_on_error=False)
            if not self._is_comm_success(comm):
                raise ConnectionError(f"No reply for '{register}' on {name}")
            return value, error

    class ReadOnlyBus(Bus):
        def write(self, *args, **kwargs):
            raise RuntimeError("Read-only bus: register writes blocked")

        def sync_write(self, *args, **kwargs):
            raise RuntimeError("Read-only bus: motor targets blocked")

    leader = safe["SO101Leader"](safe["SO101LeaderConfig"](port=HOST.leader_port(), id="leader"))
    follower = safe["SO101Follower"](safe["SO101FollowerConfig"](port=HOST.follower_port(), id="follower"))
    profile = HOST.profile_path()
    expected = {
        "leader": {name: asdict(leader.calibration[name]) for name in JOINTS},
        "follower": json.loads(profile.read_text()),
    }
    five = lambda device: {name: device.bus.motors[name] for name in JOINTS}  # noqa: E731
    buses = {
        "leader": ReadOnlyBus(leader.bus.port, five(leader)),
        "follower": (Bus if options.armed else ReadOnlyBus)(follower.bus.port, five(follower)),
    }
    mode = "torque" if options.armed else "dryrun"
    report = {
        "purpose": "five-joint teleoperation, TORQUE ON" if options.armed else "DRY RUN, nothing written to a motor",
        "options": {**options.command, "max_error": options.max_error, "armed": options.armed},
        "preflight": options.preflight.record if options.armed else None,
    }
    signal.signal(signal.SIGTERM, lambda *_: sys.exit("Stopped by SIGTERM"))

    def speak(words):  # does not wait: the control loop must keep its period
        try:
            HOST.say(words)
        except OSError:
            pass

    cues = {
        "follow": "Torque on. Move the black arm now." if options.armed else "Dry run. The white arm will not move.",
        "return": "Stop. Hands off the black arm." if options.armed else "Dry run ends.",
    }
    budget = energized_budget(options)
    # The software limit is not a hardware stop. If the program hangs, this timer still speaks.
    watchdog = threading.Timer(budget["total"] + 2, speak, ["Warning. Time limit. Pull the white power plug now."])
    watchdog.daemon = True
    released = True
    try:
        for key, bus in buses.items():
            users = HOST.port_users(bus.port)
            if users.returncode not in (0, 1) or users.stdout.strip():
                raise RuntimeError(f"{key} serial port unavailable")
            bus.connect(handshake=False)
        print("TORQUE ON: the white arm can move." if options.armed else "DRY RUN: the white arm will not move.", flush=True)
        print(f"Energized-time budget: {budget['total']:.1f} s = enable {budget['enable']:.1f} + follow "
              f"{budget['follow']:.1f} + return {budget['return']:.1f} + release {budget['release']:.1f}. "
              "Software limit only: if it fails, pull the DC plug of the white controller board.", flush=True)
        if options.armed:
            watchdog.start()
        run(buses["leader"], buses["follower"], expected, options, report, notify=lambda phase: speak(cues[phase]))
    except BaseException as error:
        report["error"] = repr(error)
        raise
    finally:
        for bus in buses.values():
            if bus.is_connected:
                bus.disconnect(disable_torque=False)
        released, verdict = release_verdict(options.armed, report)
        watchdog.cancel()  # the verdict below speaks its own warning when the release is not verified
        report.update(release_verified=released, release_verdict=verdict)
        path = ROOT / f"so101-teleop-{mode}-{datetime.now():%Y%m%d-%H%M%S-%f}.json"
        path.write_text(json.dumps(report, indent=2))
        release = report.get("release") or {}
        print(f"Result: {report.get('result', 'not started')}. Torque readback: {release.get('torque')}. "
              f"Torque limit readback: {release.get('torque_limit')}. "
              f"Torque-off passes: {len(release.get('passes') or [])}", flush=True)
        print(verdict, flush=True)
        print(f"Evidence: {path}", flush=True)
        if options.armed:
            speak("Torque is off." if released else "Warning. Pull the white power plug now.")
    if not released:
        raise SystemExit(2)


def main(argv=None, start=None):
    """`start` takes the checked options; the default opens the serial ports. The self-test gives the fake arms."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--self-test", action="store_true", help="Offline checks; never opens hardware")
    parser.add_argument("--enable-torque", action="store_true", help="Send torque and goals. Needs explicit approval and --preflight.")
    parser.add_argument("--preflight", type=Path, help="Preflight record (JSON) for this exact command. There is no override.")
    parser.add_argument("--seconds", type=float, default=10, help=f"Follow time, at most {MAX_SECONDS}")
    parser.add_argument("--speed", type=int, default=80, help="Goal speed in ticks per second, at most 400")
    parser.add_argument("--travel", type=int, default=0, help="Largest distance from the start in ticks; 0 holds")
    parser.add_argument("--torque-limit", type=int, default=500, help="100 to 1000 (1000 is full torque)")
    args = parser.parse_args(argv)
    if args.self_test:
        self_test()
        print("PASS: hold, follow, travel limit, energized-time budget (low-torque home, unreachable home, silent joints, slow enable), blocked joint, lost replies, release failure, lost torque-off write, silent joint, torque-limit restore rule, rest on the stop, error flag at enable, refusals, dry run, guard, preflight (failed clearance, missing readiness, unusable camera evidence, old checks, wrong command). No hardware opened.")
        return
    if not 0 < args.seconds <= MAX_SECONDS:
        parser.error(f"--seconds must be greater than zero and at most {MAX_SECONDS}")
    if not 10 <= args.speed <= 400:
        parser.error("--speed must be 10 to 400")
    if not 0 <= args.travel <= 4096:
        parser.error("--travel must be 0 to 4096")
    if not 100 <= args.torque_limit <= 1000:
        parser.error("--torque-limit must be 100 to 1000")
    if args.preflight and not args.enable_torque:
        parser.error("--preflight is for --enable-torque; a dry run needs no record")
    try:
        preflight = None
        if args.enable_torque:  # the only place that arms a run
            try:
                record = json.loads(args.preflight.read_text()) if args.preflight else "no record was given"
            except (OSError, ValueError) as error:
                record = repr(error)
            if isinstance(record, str):
                raise PreflightRefused(
                    "POWERED RUN REFUSED. Nothing was written to a motor.\n"
                    f"- preflight: --enable-torque needs --preflight <record.json> ({record})"
                )
            preflight = Preflight(record, datetime.now().astimezone())
        options = Options(args.seconds, args.speed, args.travel, args.torque_limit, preflight=preflight)
    except PreflightRefused as error:
        print(error, file=sys.stderr, flush=True)
        raise SystemExit(3) from None
    (start or hardware)(options)


if __name__ == "__main__":
    main()
