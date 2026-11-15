"""Scripted camera sweep for the white five-joint SO-101, built on so101-five-joint-teleop.py.

The teleoperation program stays in charge of everything that touches a motor: the preflight record, the torque
limit, the goal rate limit, the travel limit, the health checks, the return to the start pose, the verified
torque-off and the evidence file. Nothing in it is changed or bypassed. This file only replaces WHERE the motion
comes from: instead of the black arm's live positions, the follow loop is given a planned, smooth offset for each
joint. The black arm is not read and need not be powered (its USB port is still opened by the stock program, which
sends nothing to it).

Default is a DRY RUN: the loop runs against the real arms and logs the goals it would send; nothing is written.
--enable-torque needs --preflight <record.json>, exactly as in the teleoperation program. There is no override.

  python so101-five-joint-sweep.py --self-test
  python so101-five-joint-sweep.py --plan gentle --seconds 60 --speed 120 --travel 150            (dry run)
  python so101-five-joint-sweep.py --plan gentle --seconds 60 --speed 120 --travel 150 --torque-limit 500 \
         --enable-torque --preflight record.json
"""
import importlib.util
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("so101_teleop", ROOT / "so101-five-joint-teleop.py")
T = importlib.util.module_from_spec(spec)
sys.modules["so101_teleop"] = T
spec.loader.exec_module(T)
JOINTS = T.JOINTS
HOST = T.HOST                          # Mac / Linux differences: ports, speech, busy-port check
TICKS_PER_DEG = 4096 / 360

# A plan gives each joint an amplitude in degrees and a period in seconds. The offset is amplitude * sin(2 pi t / period),
# faded in over the first 3 seconds, so the arm starts and stays smooth. Different periods make the camera wander
# over many different views instead of repeating one loop. The teleoperation loop still clamps every goal to
# --travel ticks from the start pose and to --speed ticks per second.
PLANS = {
    "still": {name: (0.0, 10.0) for name in JOINTS},
    "gentle": {"shoulder_pan": (8.0, 23.0), "shoulder_lift": (4.0, 31.0), "elbow_flex": (4.0, 19.0),
               "wrist_flex": (8.0, 17.0), "wrist_roll": (12.0, 29.0)},
    # run 1 ("gentle") covered 11 and 12 of 16 image cells; this uses the whole 13 degree envelope on the aiming joints
    "medium": {"shoulder_pan": (13.0, 23.0), "shoulder_lift": (7.0, 31.0), "elbow_flex": (7.0, 19.0),
               "wrist_flex": (13.0, 17.0), "wrist_roll": (13.0, 29.0)},
    "wide": {"shoulder_pan": (18.0, 23.0), "shoulder_lift": (8.0, 31.0), "elbow_flex": (8.0, 19.0),
             "wrist_flex": (16.0, 17.0), "wrist_roll": (25.0, 29.0)},
}


def bench_keys(aim=20.0, roll_neg=180.0, roll_pos=60.0, lift=6.0):
    """A routine bench operators use by hand: a plus, a wrist rotation each way, then an X.

    (seconds, pan, lift, elbow, wrist_flex, wrist_roll) in degrees from the start pose. Pan swings the view left
    and right, wrist flex up and down, wrist roll turns the camera about its own view axis. The roll servo has one
    turn of travel, so "180 each way" is only possible from mid-range; from a rest pose near 3305 ticks it has
    287 degrees one way and 66 the other, hence 180 and 60.
    """
    a, l = aim, lift
    return [(0, 0, 0, 0, 0, 0),
            (3, -a, 0, 0, 0, 0), (7.5, a, 0, 0, 0, 0), (10, 0, 0, 0, 0, 0),                   # plus: left, right, centre
            (13, 0, l, 0, -a, 0), (17.5, 0, -l, 0, a, 0), (20, 0, 0, 0, 0, 0),                # plus: up, down, centre
            (29, 0, 0, 0, 0, -roll_neg), (40, 0, 0, 0, 0, roll_pos), (43, 0, 0, 0, 0, 0),     # rotate one way, the other, centre
            (46, -a, l, 0, -a, 0), (50, a, -l, 0, a, 0), (52, 0, 0, 0, 0, 0),                 # X: first diagonal
            (55, a, l, 0, -a, 0), (59, -a, -l, 0, a, 0), (61, 0, 0, 0, 0, 0)]                 # X: second diagonal


def bench_fast_keys(aim=22.0, lift=6.0, roll_neg=180.0, roll_pos=60.0):
    """The same plus / rotation / X at the fastest speed the controller allows (400 ticks, 35 degrees, a second),
    then a second plus with the camera rolled 90 degrees. Peak speeds: aiming joints 26 deg/s, roll 33 deg/s.
    Faster than about 33 deg/s of camera rotation the board moves more than 20 px a frame at 30 fps and blurs, so this is the top.
    """
    a, l = aim, lift
    L, R, U, D, C = (-a, 0, 0, 0), (a, 0, 0, 0), (0, l, 0, -a), (0, -l, 0, a), (0, 0, 0, 0)
    UL, DR, UR, DL = (-a, l, 0, -a), (a, -l, 0, a), (a, l, 0, -a), (-a, -l, 0, a)
    seq = [(1.5, L, 0), (2.5, R, 0), (1.5, C, 0), (1.5, U, 0), (2.5, D, 0), (1.5, C, 0),            # plus
           (8.5, C, -roll_neg), (11.0, C, roll_pos), (3.0, C, 0),                                    # rotation
           (1.5, UL, 0), (2.5, DR, 0), (1.5, C, 0), (1.5, UR, 0), (2.5, DL, 0), (1.5, C, 0),         # X
           (4.5, C, -90), (1.5, L, -90), (2.5, R, -90), (1.5, C, -90), (1.5, U, -90), (2.5, D, -90), (1.5, C, -90)]
    keys, t = [(0, 0, 0, 0, 0, 0)], 0.0
    for dt, aimv, roll in seq:
        t += dt; keys.append((t, *aimv, -roll if roll > 0 and False else roll))
    return keys


AIM_CAP_DEG = 25.0   # no plan may ask an aiming joint (pan, lift, elbow, wrist flex) for more than this, whatever --travel is
# The station may raise it for a plan designed to bring the camera close to the board (SWEEP_AIM_CAP, at most 120 degrees).
# Joint limits, --travel and the lag abort still apply.
if os.environ.get("SWEEP_AIM_CAP"): AIM_CAP_DEG = max(0.0, min(120.0, float(os.environ["SWEEP_AIM_CAP"])))
PLANS["bench"] = {"keys": bench_keys()}
PLANS["bench-fast"] = {"keys": bench_fast_keys()}
PLANS["bench-small"] = {"keys": bench_keys(aim=12.0, roll_neg=13.0, roll_pos=13.0, lift=4.0)}


def key_offsets(keys, t):
    t = max(0.0, min(t, keys[-1][0]))
    for a, b in zip(keys, keys[1:]):
        if a[0] <= t <= b[0]:
            u = (t - a[0]) / (b[0] - a[0]); u = u * u * (3 - 2 * u)                            # ease in and out
            deg = {name: a[i + 1] + (b[i + 1] - a[i + 1]) * u for i, name in enumerate(JOINTS)}
            for name in JOINTS:
                if name != "wrist_roll":
                    deg[name] = max(-AIM_CAP_DEG, min(AIM_CAP_DEG, deg[name]))
            return {name: int(round(deg[name] * TICKS_PER_DEG)) for name in JOINTS}
    return {name: 0 for name in JOINTS}


def offsets(plan, t):
    """Planned offset of each joint from its start, in ticks, at t seconds into the sweep."""
    if "keys" in plan:
        return key_offsets(plan["keys"], t)
    fade = min(1.0, max(0.0, t) / 3.0)
    return {name: int(round(fade * amp * TICKS_PER_DEG * math.sin(2 * math.pi * t / period)))
            for name, (amp, period) in plan.items()}


def peak_speed(plan):
    """Fastest planned joint speed in ticks per second."""
    if "keys" in plan:
        k = plan["keys"]
        return max(1.5 * abs(b[i] - a[i]) * TICKS_PER_DEG / (b[0] - a[0]) for a, b in zip(k, k[1:]) for i in range(1, 6))
    return max(amp * TICKS_PER_DEG * 2 * math.pi / period for amp, period in plan.values())


class ScriptedLeader:
    """Stands in for the black arm. It never talks to it: position reads return a fixed pose plus the planned offset.

    The sweep drives the white arm directly, so the black arm is not part of it and
    need not be powered. The white arm's own checks (replies, torque off, calibration readback, health, limits) and
    the preflight record are unchanged.
    """

    START = 2048

    def __init__(self, plan, clock, bias=None, lead=0.0):
        self._plan, self._clock, self._first, self._t0 = plan, clock, True, None
        self._bias, self._lead = bias or {name: 0 for name in JOINTS}, lead

    def sync_read(self, register, names, *args, **kwargs):
        if register != "Present_Position":
            raise RuntimeError(f"The sweep does not read {register} from the black arm")
        if self._first:                    # first read: the loop takes this as the leader start
            self._first = False
            return {name: self.START for name in JOINTS}
        if self._t0 is None:
            self._t0 = self._clock()
        t = self._clock() - self._t0
        off = offsets(self._plan, t - self._lead) if t >= self._lead else {name: 0 for name in JOINTS}
        u = 1.0 if self._lead <= 0 else min(1.0, t / self._lead); u = u * u * (3 - 2 * u)      # ease the arm to the home pose first
        return {name: self.START + off[name] + int(round(self._bias[name] * u)) for name in JOINTS}


HOME = None            # fixed pose (ticks) that every sweep is centred on; None centres the sweep on wherever the arm rests
HOME_MAX_TICKS = 700   # furthest the arm may be from home at the start (61 degrees); further than that, move it by hand


def plan_reach_ticks(plan):
    """How far, per joint, the plan itself takes the arm from home (ticks, both directions), after the aiming cap."""
    reach = {n: [0, 0] for n in JOINTS}
    for k in plan.get("keys") or []:
        for i, n in enumerate(JOINTS):
            d = k[i + 1] if n == "wrist_roll" else max(-AIM_CAP_DEG, min(AIM_CAP_DEG, k[i + 1]))
            t = int(round(d * TICKS_PER_DEG)); reach[n][0] = min(reach[n][0], t); reach[n][1] = max(reach[n][1], t)
    return reach


PLAN_REACH_MARGIN = 60   # ticks beyond the plan's own reach still accepted as "on the plan's path"
# The wrist joints carry only the camera and drop under its weight once torque goes off after a stopped sweep (a wrist flex
# several hundred ticks below home after a Stop at the close position is normal). They get a wider allowance; the big joints
# keep the plan's reach.
WRIST_SAG_MARGIN = 600


def home_bias(follower, plan=None):
    """Ticks from the arm's present pose to HOME, and the seconds allowed to get there.

    Without this every sweep is centred on the present pose. The arm stops about 20 ticks short of its start after
    each run, so over 20 runs the sweep centre crept 35 degrees and the board left the view.
    """
    if HOME is None:
        return None, 0.0
    present = follower.sync_read("Present_Position", JOINTS, normalize=False, num_retry=2)
    bias = {name: HOME[name] - present[name] for name in JOINTS}
    bias["wrist_roll"] = (bias["wrist_roll"] + 2048) % 4096 - 2048
    far = max(abs(v) for v in bias.values())
    if far > HOME_MAX_TICKS:
        # Further than the usual limit is accepted only when every joint is somewhere this same plan itself takes it (e.g. a sweep
        # stopped at the close position): the arm then goes back to home slowly, through space the plan already uses.
        reach = plan_reach_ticks(plan) if plan and "keys" in plan else None
        mg = {n: (WRIST_SAG_MARGIN if n in ("wrist_flex", "wrist_roll") else PLAN_REACH_MARGIN) for n in JOINTS}
        on_path = reach is not None and all(reach[n][0] - mg[n] <= -bias[n] <= reach[n][1] + mg[n] for n in JOINTS)
        if not on_path:
            raise RuntimeError(f"The arm is {far} ticks from the home pose; the limit is {HOME_MAX_TICKS}. Move it nearer by hand")
    return bias, max(1.0, far / 100.0)


SMALL = {"travel": 150, "torque_limit": 500, "speed": 120, "seconds": 75}  # largest sweep allowed with a person in reach


def sweep_refusals(record, now):
    """Why this record does not permit a powered SWEEP. An empty list permits it.

    The black arm is not part of a sweep, so its reach, the distance between the
    bases and "operator at the black arm" are dropped. What is kept is the part about people:
      nearest_person_cm >= follower_reach_cm + margin_cm      (white pan axis to the nearest part of any person)
    plus a fresh, inspected camera frame of the white arm and a fresh "clear" from the operator. No override.
    """
    from datetime import datetime
    if not isinstance(record, dict):
        return ["preflight: the record is not a JSON object"]
    reasons = []

    def age(part, key, where):
        try:
            seconds = (now - datetime.fromisoformat(part.get(key))).total_seconds()
        except (TypeError, ValueError, AttributeError):
            reasons.append(f"{where}: {key} is missing or has no time zone"); return
        if not -5 <= seconds <= T.PREFLIGHT_MAX_AGE:
            reasons.append(f"{where}: {key} is {seconds:.0f} s old; the limit is {T.PREFLIGHT_MAX_AGE} s. Do the check again")

    age(record, "checked_at", "preflight")
    command = record.get("command")
    if not isinstance(command, dict) or set(command) != set(T.COMMAND_KEYS) or any(type(v) not in (int, float) for v in command.values()):
        reasons.append(f"command: must give {', '.join(T.COMMAND_KEYS)} of the approved command as numbers")
    c = record.get("clearance")
    accepted = isinstance(c, dict) and c.get("operator_accepts_being_in_reach") is True
    waived = isinstance(c, dict) and c.get("operator_waived_camera_check") is True
    if waived:
        # The operator has said that they keep people clear of the arm themselves; no camera frame of the workspace is
        # inspected. Recorded as a waiver in the operator's own words, not as a clearance.
        if not isinstance(c.get("operator_words"), str) or not c["operator_words"].strip():
            reasons.append("clearance: operator_words is missing. Quote what the operator said")
    elif accepted:
        # The operator has said that they may stay inside the arm's reach. This is recorded
        # as what it is, not as a clearance, and it is only allowed for a small, slow, half-torque sweep.
        if not isinstance(c.get("operator_words"), str) or not c["operator_words"].strip():
            reasons.append("clearance: operator_words is missing. Quote what the operator said")
        cmd = command if isinstance(command, dict) else {}
        small = (cmd.get("travel", 9999) <= SMALL["travel"] and cmd.get("torque_limit", 9999) <= SMALL["torque_limit"]
                 and cmd.get("speed", 9999) <= SMALL["speed"] and cmd.get("seconds", 9999) <= SMALL["seconds"])
        if not small:
            reasons.append(f"clearance: a person inside the reach is only accepted for a small sweep {SMALL}. Move away for a larger one")
    elif not isinstance(c, dict):
        reasons.append("clearance: missing")
    else:
        vals = [c.get(k) for k in ("follower_reach_cm", "nearest_person_cm", "margin_cm")]
        if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in vals):
            reasons.append("clearance: follower_reach_cm, nearest_person_cm and margin_cm must be positive numbers")
        else:
            reach, person, margin = vals
            if reach < T.REACH_FLOOR_CM: reasons.append(f"clearance: follower_reach_cm {reach} is less than the model value {T.REACH_FLOOR_CM}")
            if margin < T.MIN_MARGIN_CM: reasons.append(f"clearance: margin_cm {margin} is less than {T.MIN_MARGIN_CM}")
            if person < reach + margin:
                reasons.append(f"clearance: FAILED. A person is {person} cm from the white pan axis; {reach + margin:.1f} cm are necessary. Move away from the white arm")
        if not isinstance(c.get("method"), str) or not c["method"].strip():
            reasons.append("clearance: method is missing. Say how the distance is known")
    o = record.get("operator")
    if not isinstance(o, dict) or o.get("ready") is not True:
        reasons.append("operator: the operator has not said they are clear of the white arm")
    else:
        age(o, "confirmed_at", "operator")
    cam = record.get("camera")
    if waived:
        pass
    elif not isinstance(cam, dict):
        reasons.append("camera: missing")
    else:
        age(cam, "captured_at", "camera")
        if accepted:
            if cam.get("shows_white_arm") is not True:
                reasons.append("camera: the inspected frame does not show the white arm")
        elif cam.get("shows_white_arm_and_nobody_in_reach") is not True:
            reasons.append("camera: the inspected frame does not show the white arm with nobody inside its reach")
        reasons.extend(T.frame_problems(cam, now))
    return reasons


def install(plan_name, people_preflight=True):
    """Make the teleoperation program's follow loop read the planned motion. Returns the plan."""
    plan = PLANS[plan_name]
    original = T.run

    def run(leader, follower, expected, options, report, clock=time.monotonic, sleep=time.sleep, notify=lambda phase: None):
        report["sweep"] = {"plan": plan_name, "definition": plan, "peak_speed_ticks_s": round(peak_speed(plan), 1)}
        if "purpose" in report:
            report["purpose"] = "five-joint scripted camera sweep, " + ("TORQUE ON" if options.armed else "DRY RUN, nothing written to a motor")
        words = {"follow": "Torque on. The white arm is sweeping. Keep clear." if options.armed else "Dry run. The white arm will not move.",
                 "return": "Sweep ends. Returning to start." if options.armed else "Dry run ends."}

        def cue(phase):                    # the stock cue tells the operator to move the black arm; not true here
            try:
                HOST.say(words[phase])
            except (OSError, KeyError):
                pass

        spoken = notify if getattr(notify, "__self__", None) is not None else cue   # the self-test passes list.append
        stock_ready = T.check_ready

        def ready(bus, key, expected_part):          # the black arm is not used: only the white arm is checked
            if key != "leader":
                stock_ready(bus, key, expected_part)

        T.check_ready = ready
        try:
            bias, lead = home_bias(follower, plan)
            report["sweep"].update(home=HOME, home_bias_ticks=bias, home_lead_seconds=lead)
            stock_targets = T.next_targets

            def targets_ending_at_home(leader_start, leader_now, follower_start, previous, limits, dt, speed=80, travel=None):
                # The stock return phase goes back to where the arm rested BEFORE the sweep. With a home pose the sweep must END at
                # home instead: after a Stop or a fault the arm can rest anywhere, and the next camera check needs the board in view.
                # The return phase is the call with the leader at its start and zero travel; from then on "start" means home, moved
                # inside the joint limits so that the arm can actually reach it and the stock program can see it has arrived.
                if HOME is not None and leader_now is leader_start and travel == 0 and "rest_pose_before" not in report["sweep"]:
                    report["sweep"]["rest_pose_before"] = dict(follower_start)
                    for name in JOINTS:
                        follower_start[name] = max(limits[name][0], min(limits[name][1], HOME[name]))
                return stock_targets(leader_start, leader_now, follower_start, previous, limits, dt, speed, travel)

            T.next_targets = targets_ending_at_home
            try:
                return original(ScriptedLeader(plan, clock, bias, lead), follower, expected, options, report, clock, sleep, spoken)
            finally:
                T.next_targets = stock_targets
        finally:
            T.check_ready = stock_ready

    T.run = run
    if people_preflight:
        T.preflight_refusals = sweep_refusals  # people-only preflight for sweeps; see sweep_refusals
    return plan


def white_only_hardware(options):
    """The stock hardware(), with the black arm left out entirely: its USB port is not opened, so it may be unplugged.

    Everything about the white arm is as in the stock function: same bus class, same read-only bus for a dry run,
    same serial-port check, same watchdog, same run(), same release verdict and the same evidence file.
    """
    import json, runpy, signal, subprocess, threading
    from dataclasses import asdict  # noqa: F401  (kept for parity with the stock function)
    from datetime import datetime
    safe = runpy.run_path(str(ROOT / "so101-calibrate-safe.py"))
    from lerobot.motors.motors_bus import get_address

    class Bus(safe["FeetechMotorsBus"]):
        def read_raw(self, register, name):
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

    follower = safe["SO101Follower"](safe["SO101FollowerConfig"](port=HOST.follower_port(), id="follower"))
    profile = HOST.profile_path()
    expected = {"leader": {}, "follower": json.loads(profile.read_text())}
    bus = (Bus if options.armed else ReadOnlyBus)(follower.bus.port, {name: follower.bus.motors[name] for name in JOINTS})
    mode = "torque" if options.armed else "dryrun"
    report = {"purpose": "", "options": {**options.command, "max_error": options.max_error, "armed": options.armed},
              "preflight": options.preflight.record if options.armed else None, "black_arm": "not opened"}
    def stop_once(signum, _frame):
        # Only the FIRST stop request may interrupt the sweep. A second one arriving while torque is being released would
        # interrupt the release itself (seen once: two stop signals, release left unconfirmed), so later ones are ignored.
        signal.signal(signal.SIGTERM, signal.SIG_IGN); signal.signal(signal.SIGINT, signal.SIG_IGN)
        sys.exit("Stopped by a stop request")

    signal.signal(signal.SIGTERM, stop_once); signal.signal(signal.SIGINT, stop_once)

    def speak(words):
        try:
            HOST.say(words)
        except OSError:
            pass

    budget = T.energized_budget(options)
    watchdog = threading.Timer(budget["total"] + 2, speak, ["Warning. Time limit. Pull the white power plug now."])
    watchdog.daemon = True
    released = True
    try:
        users = HOST.port_users(bus.port)
        if users.returncode not in (0, 1) or users.stdout.strip():
            raise RuntimeError("follower serial port unavailable")
        bus.connect(handshake=False)
        # Smoother motion. With the servos' Acceleration register at 0 each joint races to every new goal and
        # stops, ten times a second. The camera's gyro sees that as shake (7 deg/s), which spoils any camera-to-IMU
        # alignment made from the recording. SWEEP_ACCEL=<1..254> (units of 100 steps/s^2) makes each servo ramp instead. The old value is put back
        # before the port is closed. It is a RAM register: a power cycle clears it as well.
        import os
        accel = int(os.environ.get("SWEEP_ACCEL", "0") or 0)
        if options.armed and 1 <= accel <= 254:
            before = {n: bus.read_raw("Acceleration", n)[0] for n in T.JOINTS}
            for n in T.JOINTS: bus.write("Acceleration", n, accel, normalize=False)
            report["smoothing"] = {"acceleration_before": before, "acceleration_set": accel, "readback": {n: bus.read_raw("Acceleration", n)[0] for n in T.JOINTS}}
            print(f"Servo acceleration set to {accel} (was {sorted(set(before.values()))}).", flush=True)
        print("TORQUE ON: the white arm can move." if options.armed else "DRY RUN: the white arm will not move.", flush=True)
        print(f"Energized-time budget: {budget['total']:.1f} s. Software limit only: if it fails, pull the DC plug of the white controller board.", flush=True)
        if options.armed:
            watchdog.start()
        T.run(None, bus, expected, options, report, notify=lambda phase: None)
    except BaseException as error:
        report["error"] = repr(error)
        raise
    finally:
        if bus.is_connected and report.get("smoothing"):
            try:
                for n, v in report["smoothing"]["acceleration_before"].items(): bus.write("Acceleration", n, v, normalize=False)
                report["smoothing"]["restored"] = {n: bus.read_raw("Acceleration", n)[0] for n in T.JOINTS}
            except Exception as e:
                report["smoothing"]["restore_error"] = repr(e)
        if bus.is_connected:
            bus.disconnect(disable_torque=False)
        released, verdict = T.release_verdict(options.armed, report)
        watchdog.cancel()
        report.update(release_verified=released, release_verdict=verdict)
        path = ROOT / f"so101-teleop-{mode}-{datetime.now():%Y%m%d-%H%M%S-%f}.json"
        path.write_text(json.dumps(report, indent=2))
        release = report.get("release") or {}
        print(f"Result: {report.get('result', 'not started')}. Torque readback: {release.get('torque')}. "
              f"Torque limit readback: {release.get('torque_limit')}. Torque-off passes: {len(release.get('passes') or [])}", flush=True)
        print(verdict, flush=True)
        print(f"Evidence: {path}", flush=True)
        if options.armed:
            speak("Torque is off." if released else "Warning. Pull the white power plug now.")
    if not released:
        raise SystemExit(2)


def json_copy(x):
    import json
    return json.loads(json.dumps(x))


def self_test():
    """Offline, fake arms only (the teleoperation program's own simulator)."""
    T.self_test()                                             # the unchanged program still passes its own tests
    stock_run, stock_pre = T.run, T.preflight_refusals
    try:
        for plan_name, travel, speed in (("gentle", 150, 120), ("wide", 300, 200), ("still", 0, 80)):
            plan = install(plan_name, people_preflight=False)
            sim, leader, follower, follower_start, expected, options = T.scene(
                armed=True, seconds=20, speed=speed, travel=travel, torque_limit=500)
            report = T.go(sim, leader, follower, expected, options)
            assert "error" not in report, report.get("error")
            rows = [r for r in report["rows"] if r["phase"] == "follow"]
            reach = {n: max(abs(r["goal_raw"][n] - follower_start[n]) for r in rows) for n in JOINTS}
            steps = max(abs(b["goal_raw"][n] - a["goal_raw"][n]) for a, b in zip(report["rows"], report["rows"][1:]) for n in JOINTS)
            assert max(reach.values()) <= travel, (plan_name, reach)                      # travel limit holds
            assert steps <= int(speed * 0.1) + 1, (plan_name, steps)                      # rate limit holds
            if plan_name != "still":
                assert all(v > 0 for v in reach.values()), (plan_name, reach)             # every joint actually moves
                want = {n: min(travel, int(a * TICKS_PER_DEG)) for n, (a, _) in plan.items()}
                assert all(reach[n] >= 0.6 * want[n] for n in JOINTS), (plan_name, reach, want)
            assert report["result"] in ("complete", "settled short of start"), report["result"]
            released, verdict = T.release_verdict(True, report)
            assert released, verdict                                                       # torque off, verified
            assert report["sweep"]["plan"] == plan_name
            T.run = stock_run; T.preflight_refusals = stock_pre
        # a black arm that does not answer at all does not matter, and is never read
        class Dead:
            def __getattr__(self, name):
                raise AssertionError(f"the sweep touched the black arm ({name})")
        install("gentle", people_preflight=False)
        sim, leader, follower, follower_start, expected, options = T.scene(armed=True, seconds=10, speed=120, travel=150, torque_limit=500)
        report = T.go(sim, Dead(), follower, expected, options)
        assert "error" not in report, report.get("error")
        assert T.release_verdict(True, report)[0]
        T.run = stock_run; T.preflight_refusals = stock_pre
        # the white arm's own readiness check still stops a run: torque already on at the start is refused
        install("gentle", people_preflight=False)
        sim, leader, follower, follower_start, expected, options = T.scene(armed=True, seconds=5, speed=120, travel=150, torque_limit=500)
        follower.torque["elbow_flex"] = 1
        report = T.go(sim, Dead(), follower, expected, options)
        T.run = stock_run; T.preflight_refusals = stock_pre
        white_check = "error" in report
        # people-only preflight: near person refused, stale check refused, missing frame claim refused, good record accepted
        import tempfile, hashlib
        from datetime import datetime, timedelta
        from PIL import Image
        import random
        now = datetime.now().astimezone(); tmp = Path(tempfile.mkdtemp()) / "frame.jpg"
        rnd = random.Random(1); img = Image.new("L", (800, 450)); img.putdata([rnd.randrange(256) for _ in range(800 * 450)]); img.save(tmp)
        good = {"checked_at": now.isoformat(), "command": {"seconds": 60, "speed": 120, "travel": 150, "torque_limit": 500},
                "clearance": {"follower_reach_cm": 50, "nearest_person_cm": 100, "margin_cm": 20, "method": "stated by the operator"},
                "operator": {"ready": True, "confirmed_at": now.isoformat()},
                "camera": {"frame": str(tmp), "sha256": hashlib.sha256(tmp.read_bytes()).hexdigest(), "captured_at": now.isoformat(),
                           "shows_white_arm_and_nobody_in_reach": True}}
        assert sweep_refusals(good, now) == [], sweep_refusals(good, now)
        near = json_copy(good); near["clearance"]["nearest_person_cm"] = 40
        assert any("FAILED" in r for r in sweep_refusals(near, now))
        assert sweep_refusals(good, now + timedelta(seconds=400))                      # stale
        blind = json_copy(good); blind["camera"]["shows_white_arm_and_nobody_in_reach"] = False
        assert sweep_refusals(blind, now)
        notready = json_copy(good); notready["operator"]["ready"] = False
        assert sweep_refusals(notready, now)
        acc = json_copy(good); acc["clearance"] = {"operator_accepts_being_in_reach": True, "operator_words": "its fine, continue"}
        acc["camera"].pop("shows_white_arm_and_nobody_in_reach"); acc["camera"]["shows_white_arm"] = True
        assert sweep_refusals(acc, now) == [], sweep_refusals(acc, now)
        big = json_copy(acc); big["command"]["travel"] = 300
        assert sweep_refusals(big, now)                                              # in reach is refused for a larger sweep
        hard = json_copy(acc); hard["command"]["torque_limit"] = 1000
        assert sweep_refusals(hard, now)
        mute = json_copy(acc); mute["clearance"].pop("operator_words")
        assert sweep_refusals(mute, now)
        # a fixed home pose: the sweep is centred on home, not on where the arm happens to rest
        global HOME
        install("gentle", people_preflight=False)
        sim, leader, follower, follower_start, expected, options = T.scene(armed=True, seconds=20, speed=200, travel=600, torque_limit=500)
        HOME = {n: follower_start[n] + (250 if n == "shoulder_pan" else 0) for n in JOINTS}
        try:
            report = T.go(sim, Dead(), follower, expected, options)
        finally:
            HOME = None
        assert "error" not in report, report.get("error")
        fr = [r for r in report["rows"] if r["phase"] == "follow" and r["elapsed_seconds"] > 8]
        centre = sum(r["goal_raw"]["shoulder_pan"] - follower_start["shoulder_pan"] for r in fr) / len(fr)
        assert 200 <= centre <= 300, centre
        assert report["sweep"]["home_bias_ticks"]["shoulder_pan"] == 250 and T.release_verdict(True, report)[0]
        # the sweep ends AT home, not back where the arm rested before it
        end = report["rows"][-1]["follower_raw"]["shoulder_pan"] - report["sweep"]["rest_pose_before"]["shoulder_pan"]
        assert 230 <= end <= 270, end
        assert report["result"] in ("complete", "settled short of start"), report["result"]
        assert report["sweep"]["rest_pose_before"]["shoulder_pan"] == follower_start["shoulder_pan"]
        HOME = {n: follower_start[n] + (900 if n == "shoulder_pan" else 0) for n in JOINTS}
        install("gentle", people_preflight=False)
        sim, leader, follower, follower_start, expected, options = T.scene(armed=True, seconds=5, speed=200, travel=600, torque_limit=500)
        try:
            report = T.go(sim, Dead(), follower, expected, options)
        finally:
            HOME = None
        assert "error" in report and not report.get("motor_writes"), "a far home pose must be refused before any motor write"
        T.run = stock_run; T.preflight_refusals = stock_pre
        wv = json_copy(good); wv["clearance"] = {"operator_waived_camera_check": True, "operator_words": "I keep people clear of the arm myself"}; wv.pop("camera")
        assert sweep_refusals(wv, now) == [], sweep_refusals(wv, now)
        assert sweep_refusals(wv, now + timedelta(seconds=400))                        # still expires
        nw = json_copy(wv); nw["clearance"].pop("operator_words"); assert sweep_refusals(nw, now)
        # an unarmed run writes nothing
        install("gentle", people_preflight=False)
        sim, leader, follower, follower_start, expected, options = T.scene(seconds=5, speed=120, travel=150)
        report = T.go(sim, leader, follower, expected, options)
        assert "error" not in report and not report["motor_writes"], report.get("motor_writes")
        # without a preflight record an armed run is refused before anything else happens
        try:
            T.Options(10, 120, 150, 500, preflight="not a record")
            raise AssertionError("an unchecked preflight armed a run")
        except T.PreflightRefused:
            pass
    finally:
        T.run = stock_run; T.preflight_refusals = stock_pre
    print("PASS: teleoperation self-test unchanged; gentle, wide and still sweeps stay inside the travel and rate limits, "
          "every joint moves, the arm returns and torque-off is verified; a dry run writes nothing; "
          "a run cannot be armed without a checked preflight record; the people-only preflight refuses a near person, a stale check, an unconfirmed frame and a not-ready operator, and accepts a person in reach only when the operator says so and only for a small half-torque sweep; the black arm is never read"
          + ("; the white arm readiness check still refuses a bad start" if white_check else "; (white readiness negative test not available in this simulator)")
          + ". No hardware opened.")


MAX_SPEED_TICKS = 700   # 61 degrees a second. The stock program stops at 400.


def main():
    argv = sys.argv[1:]
    if "--self-test" in argv:
        self_test()
        return
    plan_name = "gentle"
    if "--plan" in argv:
        i = argv.index("--plan"); plan_name = argv[i + 1]; del argv[i:i + 2]
    if "--home" in argv:                            # five raw positions: pan,lift,elbow,wrist_flex,wrist_roll
        global HOME
        i = argv.index("--home"); vals = [int(v) for v in argv[i + 1].split(",")]; del argv[i:i + 2]
        if len(vals) != 5 or not all(0 <= v <= 4095 for v in vals):
            sys.exit("--home needs five tick values 0..4095")
        HOME = dict(zip(JOINTS, vals))
    if "--plan-file" in argv:                       # {"keys": [[seconds, pan, lift, elbow, wrist_flex, wrist_roll], ...]} in degrees
        import json
        i = argv.index("--plan-file"); path = Path(argv[i + 1]); del argv[i:i + 2]
        keys = [tuple(float(v) for v in k) for k in json.loads(path.read_text())["keys"]]
        if any(len(k) != 6 for k in keys) or keys[0] != (0, 0, 0, 0, 0, 0) or any(v != 0 for v in keys[-1][1:]) \
                or any(b[0] <= a[0] for a, b in zip(keys, keys[1:])):
            sys.exit("--plan-file: keys must be 6 numbers each, start at all zeros, rise in time and end at zero offsets")
        plan_name = path.parent.name + "/" + path.stem
        PLANS[plan_name] = {"keys": keys}
    if plan_name not in PLANS:
        sys.exit(f"--plan must be one of {', '.join(PLANS)}")
    plan = install(plan_name)
    print(f"Sweep plan '{plan_name}': fastest planned joint speed {peak_speed(plan):.0f} ticks per second "
          "(the loop limits goals to --speed and --travel).", flush=True)
    speed = int(argv[argv.index("--speed") + 1]) if "--speed" in argv else 80
    if speed <= 400:
        T.main(argv, start=white_only_hardware)
        return
    # Above the stock limit: the same steps as the stock main(), with only the speed bound changed.
    import argparse, json
    from datetime import datetime
    if speed > MAX_SPEED_TICKS:
        sys.exit(f"--speed must be at most {MAX_SPEED_TICKS}")
    ap = argparse.ArgumentParser()
    ap.add_argument("--enable-torque", action="store_true"); ap.add_argument("--preflight", type=Path)
    ap.add_argument("--seconds", type=float, default=10); ap.add_argument("--speed", type=int)
    ap.add_argument("--travel", type=int, default=0); ap.add_argument("--torque-limit", type=int, default=500)
    args = ap.parse_args(argv)
    if not 0 < args.seconds <= T.MAX_SECONDS or not 0 <= args.travel <= 4096 or not 100 <= args.torque_limit <= 1000:
        sys.exit("--seconds, --travel or --torque-limit is outside the stock limits")
    if args.preflight and not args.enable_torque:
        sys.exit("--preflight is for --enable-torque")
    try:
        preflight = None
        if args.enable_torque:
            try:
                record = json.loads(args.preflight.read_text()) if args.preflight else "no record was given"
            except (OSError, ValueError) as error:
                record = repr(error)
            if isinstance(record, str):
                raise T.PreflightRefused(f"POWERED RUN REFUSED. Nothing was written to a motor.\n- preflight: needs --preflight <record.json> ({record})")
            preflight = T.Preflight(record, datetime.now().astimezone())
        options = T.Options(args.seconds, args.speed, args.travel, args.torque_limit, preflight=preflight)
    except T.PreflightRefused as error:
        print(error, file=sys.stderr, flush=True)
        raise SystemExit(3) from None
    white_only_hardware(options)


if __name__ == "__main__":
    main()
