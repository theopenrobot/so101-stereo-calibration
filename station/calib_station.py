#!/usr/bin/env python3
"""Calibration station: a person fits a camera on the arm, presses Start, the arm records A and B, the solver gives a verdict,
the person swaps the camera. The station waits as long as it takes between cameras and never moves the arm unless Start was
pressed (or the one automatic start, below).

    <python> calib_station.py            the operator page is calib_dashboard.py: http://127.0.0.1:18771/
    API on 127.0.0.1:18772 (STATION_PORT):  GET /state   POST /start   POST /stop   POST /sethome

One path, no fallback (ARCHITECTURE.md): picture check, then the arm records A then B; then, with the arm stopped, the
open solver (stereo_solver.py) runs its first check on A alone, solves the pair, and the banding check runs on A. The
verdict, the calibration file and the recordings stay in the data folder. Nothing is written to the camera and nothing is
sent anywhere.

States
  no_camera   nothing plugged in. Waits.
  checking    a camera appeared: read its id, pick the exposure, check the board is in view.
  not_ready   the check failed (board not in view, banding in a still picture). Re-checks every few seconds.
  ready       waiting for a person to press Start.
  recording_a / recording_b / first_check / solving / qc
  passed / failed / stopped / error      result shown; waits for Start (run the same camera again) or for the camera to be unplugged.

Safety: the arm only moves between Start and the end of recording B. Stop ends the sweep at once; the arm program then
releases torque and reads it back. Unplugging the camera or losing the picture during a sweep also stops it, and so does
losing contact with the Stop button when buttons are enabled.
"""
import json, math, os, re, signal, subprocess, sys, threading, time, urllib.error, urllib.request
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import hostconf as H                                      # noqa: E402
import sessions as SES                                    # noqa: E402

PY, CAM, IN, DATA = H.PY, H.CAMERA_URL, H.ARM_DIR, H.DATA
STATION = DATA / "station"; STATION.mkdir(parents=True, exist_ok=True)
CONFIG = STATION / "config.json"
LOG = STATION / "log.jsonl"
# Who this station is, sent with every command to the camera program (it obeys one station at a time). Child programs (the
# sweep, the exposure picker) inherit it through the environment.
STATION_ID = os.environ.setdefault("STATION_ID", f"{H.STATION_NAME}@{os.uname().nodename}")
CAM_HDR = {"X-Station": STATION_ID}
AUTO_FILE = STATION / "auto_started.json"          # cameras that have had their one automatic start (kept across restarts)
DEFECTIVE = STATION / "defective.json"             # cameras marked defective by the operator: {id: {"reason": ...}}; never started automatically
DEFAULT = {"home": None,                           # five raw servo positions; set with POST /sethome (the page's SET HOME button)
           "plan": "plans/close2/plan.json", "seconds": None, "speed": 400, "travel": 2100, "torque_limit": 800, "accel": 0,
           "aim_cap_deg": 100,                     # the close plans turn the aiming joints up to 95 degrees from home (the arm program allows 120)
           "min_sharpness": 300, "exposure_hours": 3, "exposure_fixed_us": None, "require_eye_size": None,
           "still_banding_limit": 0.6, "auto_start": True,
           # Push buttons on a Pi's pin header, read by the camera program. stop_normally_closed: wire the STOP button so the
           # circuit is closed at rest and opens when pressed; then a broken wire reads as Stop.
           "buttons": {"enabled": False, "stop_normally_closed": False}}

S = {"phase": "no_camera", "message": "Fit a camera on the arm and plug it in.", "camera": None, "since": time.time(), "exposure_us": None,
     "check": None, "result": None, "history": [], "step": "", "run_started": None, "config_problem": None, "solver": "opencv"}
lock = threading.RLock()
want_start = threading.Event(); want_stop = threading.Event()
proc = {"p": None}
_good = {"conf": None}


def cfg():
    """Settings from station/config.json over the defaults. A file that cannot be read is never replaced by the defaults
    silently: the last good settings stay in use and the page shows the problem. No file at all: the defaults."""
    try:
        c = {**DEFAULT, **(json.loads(CONFIG.read_text()) if CONFIG.exists() else {})}; _good["conf"] = c; S["config_problem"] = None; return c
    except (OSError, ValueError) as e:
        S["config_problem"] = f"station/config.json could not be read ({type(e).__name__}); using the last good settings"
        if _good["conf"] is None: raise RuntimeError("station/config.json could not be read and there are no earlier settings") from e
        return dict(_good["conf"])


def write_json(path, data):
    """Replace a file in one step, so a reader never sees it half written."""
    tmp = path.with_suffix(path.suffix + ".tmp"); tmp.write_text(json.dumps(data, indent=1)); os.replace(tmp, path)


def read_json(path, default):
    try: return json.loads(path.read_text())
    except (OSError, ValueError): return default


def cam_post(path, data=None, timeout=12):
    h = {**CAM_HDR, **({"Content-Type": "application/json"} if data is not None else {})}
    return json.loads(urllib.request.urlopen(urllib.request.Request(CAM + path, data=data, method="POST", headers=h), timeout=timeout).read())


def cam_get(path, timeout=6):
    return json.loads(urllib.request.urlopen(CAM + path, timeout=timeout).read())


LIGHTS = {"no_camera": "waiting", "checking": "busy", "not_ready": "problem", "ready": "ready", "recording_a": "moving", "recording_b": "moving",
          "first_check": "busy", "solving": "busy", "qc": "busy", "passed": "pass", "failed": "fail", "stopped": "ready", "error": "problem"}
BUSY = ("recording_a", "recording_b", "first_check", "solving", "qc")
CAN_START = ("ready", "passed", "failed", "stopped", "error")


def lights(phase):
    """Show the station's state on the camera machine's lights (a Pi's two board lights). Never allowed to hold anything up."""
    def go():
        try: cam_post(f"/led?state={LIGHTS.get(phase, 'off')}", timeout=3)
        except Exception: pass
    threading.Thread(target=go, daemon=True).start()


def set_phase(phase, message, speak=None, **more):
    with lock:
        if S["phase"] != phase: S["since"] = time.time()
        S.update(phase=phase, message=message, **more)
    lights(phase)
    if speak: H.say(speak)


def camera():
    """("gone", None) when no camera device is there; ("no_picture", c or None) when a device is there but no live picture
    arrives, or the camera program did not answer; ("ok", c) otherwise. Only "gone" counts as unplugged."""
    try: c = cam_get("/camera", 10)
    except Exception: return "no_picture", None
    if not c.get("present"): return "gone", None
    if c.get("frame_age_s") is None or c["frame_age_s"] > 2.0: return "no_picture", c
    return "ok", c


EXPOSURE_FILE = STATION / "exposure.json"      # the exposure last picked by a full sweep, and when


def check_camera(c):
    """Pick the exposure and confirm the board is in view. Returns (ok, details)."""
    # The exposure depends on the light at the station, not on the camera, so the full sweep of exposures runs only every few
    # hours (exposure_hours). In between, each camera is set to the saved exposure and only checked; a board that is a fifth
    # brighter or darker than when the exposure was picked means the light changed, and the full sweep runs again.
    conf = cfg(); saved = read_json(EXPOSURE_FILE, None)
    fixed = conf.get("exposure_fixed_us")
    if fixed: saved = {"us": int(fixed), "at": time.time(), "time": time.strftime("%Y-%m-%dT%H:%M:%S%z") + " fixed in config"}
    reuse = bool(saved) and time.time() - saved.get("at", 0) < 3600 * conf.get("exposure_hours", 3)
    def pick(only):
        out = subprocess.run([PY, str(HERE / "pick_exposure.py")] + (["--only", str(only)] if only else []), capture_output=True, text=True, timeout=90)
        try: return json.loads(out.stdout.strip().splitlines()[-1]), ""
        except (ValueError, IndexError): return None, out.stderr.strip()[-200:] or "no output"
    d, err = pick(saved["us"]) if reuse else (None, "")
    white = lambda t: min(e.get("board_white") or 0 for e in t["eyes"])
    if d and d.get("chosen") and saved.get("board_white") and abs(white(d["chosen"]) - saved["board_white"]) > 0.2 * saved["board_white"]:
        d = None
    if fixed and not (d and d.get("chosen")):
        seen = max((min(e["corners"] for e in t["eyes"]) for t in (d or {}).get("tried", [])), default=0)
        return False, {"problem": f"With the fixed exposure ({int(fixed) / 1000:g} ms) the board is not fully seen (at best {seen} corners in the weaker lens)."}
    if not (d and d.get("chosen")):                # no saved exposure, it is old, or the board was not seen with it: do the full sweep
        reuse = False; d, err = pick(None)
        if d and d.get("chosen"):
            saved = {"us": d["chosen"]["us"], "at": time.time(), "time": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "board_white": white(d["chosen"]), "tried": d["tried"]}
            write_json(EXPOSURE_FILE, saved)
    if d is None: return False, {"problem": "The picture check did not run: " + err}
    if d.get("error"): return False, {"problem": d["error"][0].upper() + d["error"][1:] + "."}
    best = d.get("chosen")
    if not best:
        seen = max((min(e["corners"] for e in t["eyes"]) for t in d["tried"]), default=0)
        return False, {"problem": f"The board is not fully in view (at best {seen} of {d.get('of', '?')} corners in the weaker lens). Check the camera is seated and the board is in front of the arm.", "tried": d["tried"]}
    sharp = min(e["sharpness"] for e in best["eyes"]); size = best["size"]
    det = {"exposure_picked": saved["time"], "exposure_reused": reuse, "exposure_us": best["us"], "corners": [e["corners"] for e in best["eyes"]], "sharpness": [e["sharpness"] for e in best["eyes"]], "size": size}
    want = conf.get("require_eye_size")
    if want and list(size) != list(want):
        det["problem"] = f"This camera delivers {size[0]} x {size[1]} per lens; this station is set to require {want[0]} x {want[1]}."; return False, det
    # Banding on still live frames, before any recording: a moving scene and video compression hide stripes in a recording that
    # a still picture shows. Every camera's still score is kept with its picture check, so the limit can be set from real numbers.
    bd = still_banding()
    det["banding_still"] = bd
    lim = float(conf.get("still_banding_limit", 0.6))
    override = c.get("identity") in read_json(STATION / "qc_override.json", [])      # the operator chose to run it anyway
    if override: det["banding_override"] = True
    if bd.get("left") is not None and max(bd["left"], bd["right"]) > lim and not override:
        eye = "left" if bd["left"] >= bd["right"] else "right"
        det["problem"] = (f"QC: horizontal banding in the {eye} lens (still-picture score {max(bd['left'], bd['right']):.2f}, limit {lim:.2f}; "
                          f"left {bd['left']:.2f}, right {bd['right']:.2f}). This is a camera fault, not a calibration problem: set the camera aside.")
        return False, det
    if sharp < conf["min_sharpness"]:
        det["warning"] = f"The picture is soft (score {sharp}). Fewer frames will be usable. Put more light on the front of the board."
    return True, det


def still_banding(n=6):
    """Banding score per lens (qc_banding.frame_score, median of n still live frames), or {"error": ...}."""
    import qc_banding, numpy as np, cv2
    L, R = [], []
    for _ in range(n):
        try:
            g = cv2.imdecode(np.frombuffer(urllib.request.urlopen(CAM + "/frame.jpg", timeout=6).read(), np.uint8), 0); w = g.shape[1] // 2
            L.append(qc_banding.frame_score(g[:, :w])); R.append(qc_banding.frame_score(g[:, w:]))
        except Exception as e: return {"error": f"{type(e).__name__}: {str(e)[:80]}"}
        time.sleep(0.25)
    return {"left": round(float(np.median(L)), 3), "right": round(float(np.median(R)), 3), "frames": n}


def plan_seconds(conf):
    try: return math.ceil(json.loads((HERE / conf["plan"]).read_text())["keys"][-1][0]) + 3
    except (OSError, ValueError, KeyError, IndexError): return 30


def run_sweep(letter, serial, source):
    """One recording. Returns (ok, run folder name or None, reason, torque released)."""
    conf = cfg(); label = "c" + re.sub(r"[^A-Za-z0-9]", "", serial)[-6:] + letter
    secs = int(conf.get("seconds") or plan_seconds(conf))
    subprocess.run([PY, str(HERE / "make_preflight_waived.py"), str(DATA / "preflight-run.json")], env={**os.environ, "SECONDS_CMD": str(secs), "SPEED_CMD": str(conf["speed"]), "TRAVEL_CMD": str(conf["travel"]), "TORQUE_CMD": str(conf["torque_limit"])}, check=True)
    rec = json.loads((DATA / "preflight-run.json").read_text())
    rec["clearance"].update(operator_words=f"Started by: {source}. The operator keeps people clear of the arm.")
    (DATA / "preflight-run.json").write_text(json.dumps(rec, indent=1))
    before = {p.name for p in (DATA / "runs").glob(f"*-{label}")}
    cmd = [PY, str(HERE / "run_sweep.py"), "--plan-file", str(HERE / conf["plan"]), "--home", ",".join(map(str, conf["home"])), "--seconds", str(secs),
           "--speed", str(conf["speed"]), "--travel", str(conf["travel"]), "--torque-limit", str(conf["torque_limit"]), "--enable-torque",
           "--preflight", str(DATA / "preflight-run.json"), "--label", label]
    env = {**os.environ, "SWEEP_ACCEL": str(int(conf.get("accel", 0) or 0))}
    if conf.get("aim_cap_deg"): env["SWEEP_AIM_CAP"] = str(float(conf["aim_cap_deg"]))
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env); proc["p"] = p
    lines = []; t = threading.Thread(target=lambda: lines.extend(p.stdout), daemon=True); t.start()
    reason = ""
    last_check = t_begin = time.time(); bad = 0
    while p.poll() is None:
        if want_stop.is_set() and not reason:
            reason = "Stop was pressed"; p.send_signal(signal.SIGTERM)
        # The camera was unplugged or its picture stopped: end the sweep. Not judged in the first 12 s (the recording is being
        # started, and the recording step itself refuses to start the arm unless frames are arriving), and only while the arm
        # runs: once the arm program reports torque off, a late picture is not a reason to stop anything.
        arm_done = any("Torque reads off" in l or l.startswith("Evidence:") for l in list(lines))
        if not reason and not arm_done and time.time() - last_check > 1.0 and time.time() - t_begin > 12:
            last_check = time.time()
            try: age = cam_get("/status", 3).get("frame_age_s")
            except Exception: age = None
            bad = bad + 1 if (age is None or age > 2.5) else 0
            if bad >= 3:                           # three checks in a row, a second apart: one slow answer is not a lost camera
                reason = "the camera picture was lost during the sweep"; p.send_signal(signal.SIGTERM)
        time.sleep(0.2)
    t.join(timeout=5); proc["p"] = None
    new = sorted({q.name for q in (DATA / "runs").glob(f"*-{label}")} - before)
    text = "".join(lines)
    # Torque is off when the arm program read it back off, or when it stopped before writing anything to a motor.
    released = "Torque reads off" in text or "Stopped before any motor write" in text or "TORQUE ON" not in text
    own = [l.strip() for l in text.splitlines() if "Error" in l or "REFUSED" in l or l.startswith("- ")]
    if new:                                    # the arm finished but the recording failed its own check: say what it found
        summ = read_json(DATA / "runs" / new[-1] / "recording" / "device.json", {}).get("summary") or {}
        if summ.get("verdict") not in (None, "PASS") and summ.get("checks"):
            own.append("the camera's recording failed its own check: " + "; ".join(str(c).strip() for c in summ["checks"]))
    # The arm program's own refusal or error comes first; a later error from the recording wrapper is a consequence, not the cause.
    first = next((l for l in own if l.startswith("RuntimeError") or "REFUSED" in l or "the camera's recording failed" in l), None)
    if p.returncode != 0 and own and reason != "Stop was pressed": reason = (first or own[-1])[:200]
    elif p.returncode != 0 and not reason: reason = "the arm program stopped"
    return p.returncode == 0, (new[-1] if new else None), reason, released


def recorded_exposure(run):
    """Median exposure written in a recording's frames.csv (from frame 15), or None when it was not recorded."""
    import csv, statistics
    try:
        with open(DATA / "runs" / run / "recording" / "frames.csv") as f:
            v = [int(r["exposure_us"]) for i, r in enumerate(csv.DictReader(f)) if i >= 15 and (r.get("exposure_us") or "").isdigit()]
        return round(statistics.median(v)) if v else None
    except (OSError, ValueError, KeyError): return None


def solver(args, out, timeout):
    """Run stereo_solver.py; returns its result (the last JSON line), or {"error": ...}. Stop ends it."""
    log = open(out / f"solver-{args[0]}.log", "w")
    p = subprocess.Popen([PY, str(HERE / "stereo_solver.py"), *args, "--out", str(out)], stdout=subprocess.PIPE, stderr=log, text=True)
    proc["p"] = p; t0 = time.time()
    while p.poll() is None:
        if want_stop.is_set(): p.terminate(); p.wait(10); proc["p"] = None; return {"error": "Stop was pressed", "stopped": True}
        if time.time() - t0 > timeout: p.kill(); proc["p"] = None; return {"error": f"the solver did not finish in {timeout} s"}
        time.sleep(0.3)
    proc["p"] = None; log.close()
    try: return json.loads(p.stdout.read().strip().splitlines()[-1])
    except (ValueError, IndexError):
        tail = (out / f"solver-{args[0]}.log").read_text().strip().splitlines()[-1:] or ["no output"]
        return {"error": "the solver stopped: " + tail[0][:200]}


def calibrate(serial, source):
    """One session for the camera on the arm, on the one path (ARCHITECTURE.md). Sequential: the arm records A and B first;
    only then does anything else run, so nothing competes with the arm and the recorder for the processor."""
    conf = cfg(); want_stop.clear(); t0 = time.time(); runs = {}; timing = {}; expo = {}
    used_auto(serial)                                  # any start uses up this camera's automatic start
    with lock: S.update(result=None, run_started=t0, timing={})
    if not conf.get("home") or len(conf["home"]) != 5:
        return finish("error", "No home pose is set. Put the arm by hand where the camera looks at the middle of the board, press SET HOME, then Start.",
                      serial, t0, {"reason": "no home pose"}, speak="Station problem.")
    want_us = S.get("exposure_us")
    if want_us:                                        # the exposure this camera was checked at, set again before the arm moves
        try: r = cam_post(f"/exposure?us={int(want_us)}")
        except Exception as e: r = {"ok": False, "error": type(e).__name__}
        if not r.get("ok"):
            return finish("error", "The camera's exposure could not be set (" + str(r.get("error") or r.get("reply") or "") + "). Nothing was run.", serial, t0, {"reason": "exposure not set"}, speak="Station problem.")
    for letter, phase in (("A", "recording_a"), ("B", "recording_b")):
        set_phase(phase, f"Recording {letter}. The arm is moving: keep clear.", speak=f"Recording {letter}. Keep clear." if letter == "A" else "Recording B.")
        t_rec = time.time()
        ok, run, reason, released = run_sweep(letter, serial, source)
        timing["recording_" + letter.lower()] = round(time.time() - t_rec)
        with lock: S["timing"] = dict(timing)
        if not ok:
            stopped = want_stop.is_set() and reason == "Stop was pressed"
            msg = ("Stopped. " if stopped else f"The arm stopped during recording {letter}: {reason}. ") + ("Torque is off." if released else "CHECK THE ARM: torque-off was not confirmed. Pull the arm's power plug if it is holding.")
            return finish("stopped" if stopped else "error", msg, serial, t0, {"reason": reason, "torque_off_confirmed": released, "timing": timing, "runs": runs, "exposure_recorded_us": expo},
                          speak="Stopped. Torque is off." if released else "Warning. Check the arm.")
        runs[letter] = run
        expo[letter] = recorded_exposure(run)
        if want_us and expo[letter] is not None and abs(expo[letter] - want_us) > max(100, 0.05 * want_us):
            return finish("error", f"Recording {letter} was made at exposure {expo[letter] / 1000:g} ms, not the {want_us / 1000:g} ms it was checked at, so it is not used. Press Start to record again.",
                          serial, t0, {"reason": f"exposure in recording {letter} was {expo[letter] / 1000:g} ms, set {want_us / 1000:g} ms", "timing": timing, "runs": runs, "exposure_recorded_us": expo}, speak="Exposure wrong. Not used.")
    out = STATION / "results" / f"{re.sub(r'[^A-Za-z0-9_.-]', '_', serial)}-{datetime.now():%Y%m%d-%H%M%S}"; out.mkdir(parents=True)
    res = {"runs": runs, "timing": timing, "exposure_recorded_us": expo, "result_dir": str(out)}
    rec = lambda letter: str(DATA / "runs" / runs[letter])
    def fail(why, phase="failed", speak="Fail."):
        res.update(why=[why] if isinstance(why, str) else why, timing=timing)
        return finish(phase, ("FAIL: " if phase == "failed" else "") + "; ".join(res["why"]) + ". Press Start to try this camera again, or swap it.", serial, t0, {**res, "pass": False}, speak=speak)
    t = time.time(); set_phase("first_check", "Checking recording A.", speak="Checking.")
    fc = solver(["check", rec("A")], out, 600)
    timing["first_check"] = round(time.time() - t)
    with lock: S["timing"] = dict(timing)
    if fc.get("stopped"): return finish("stopped", "Stopped during the first check. The recordings are kept.", serial, t0, {**res, "reason": "Stop was pressed"}, speak="Stopped.")
    if fc.get("error"): return fail("the first check could not run (" + fc["error"] + ")", "error", "Station problem.")
    res["first_check"] = {k: fc.get(k) for k in ("pass", "failed", "why", "seconds")}
    if not fc.get("pass"):                             # recording A alone already fails: B cannot change that
        return fail(["recording A alone already fails: " + "; ".join(fc.get("why") or fc.get("failed") or [])], speak="Fail. " + (fc.get("why") or ["first check"])[0])
    t = time.time(); set_phase("solving", "Working out the calibration.", speak="Solving.")
    sol = solver(["solve", rec("A"), rec("B")], out, 900)
    timing["solving"] = round(time.time() - t)
    with lock: S["timing"] = dict(timing)
    if sol.get("stopped"): return finish("stopped", "Stopped during the solve. The recordings are kept.", serial, t0, {**res, "reason": "Stop was pressed"}, speak="Stopped.")
    if sol.get("error"): return fail("the solve could not run (" + sol["error"] + ")", "error", "Station problem.")
    ho = sol.get("holdout_B") or {}; st = sol.get("stereo") or {}
    checks = sol.get("checks") or []
    res.update(solve={k: sol.get(k) for k in ("pass", "failed", "why", "seconds", "calibration_file")},
               lens_error_px=[(ho.get("left") or {}).get("median_px"), (ho.get("right") or {}).get("median_px")],
               rect_dy_px=(st.get("rect_dy_B") or {}).get("median_px"), baseline_mm=st.get("baseline_mm"),
               repeat_mm=(st.get("repeat") or {}).get("translation_change_mm"), cells=[(sol.get("lenses_A") or {}).get(e, {}).get("cells") for e in ("left", "right")],
               checks_passed=f"{sum(c['pass'] for c in checks)}/{len(checks)}")
    t = time.time(); set_phase("qc", "Banding check on recording A.")
    import qc_banding
    try: qc = qc_banding.check_video(str(next(p for p in (Path(rec("A")) / "recording").glob("video.*"))))
    except Exception as e: qc = {"pass": None, "error": f"{type(e).__name__}: {str(e)[:100]}"}
    timing["qc"] = round(time.time() - t); res["qc_banding"] = qc
    with lock: S["timing"] = dict(timing)
    why = list(sol.get("why") or [])
    if qc.get("pass") is False: why.append(f"QC: horizontal stripes (banding) in the picture (left {qc.get('left')}, right {qc.get('right')}, limit {qc.get('limit')}). This is a camera fault, not a calibration problem: set the camera aside")
    elif qc.get("pass") is None: why.append("the banding check could not run (" + str(qc.get("error")) + ")")       # a check that did not run is not a pass
    if why or not sol.get("pass"):
        return fail(why or ["the solver gave no verdict"], speak="Fail. " + (why or ["no verdict"])[0])
    finish("passed", f"PASS. Calibration saved in {out.name}. Unplug the camera and fit the next one.", serial, t0, {**res, "pass": True, "why": []},
           speak="Pass. Swap the camera.")


def finish(phase, message, serial, t0, res, speak=None):
    now = datetime.now().astimezone()
    entry = {"session": f"{serial}-{now:%Y%m%d-%H%M%S}", "time": now.isoformat(timespec="seconds"), "camera": serial, "outcome": phase, "seconds": round(time.time() - t0),
             "exposure_us": S.get("exposure_us"), "station_id": STATION_ID,
             "picture_check": {k: (S.get("check") or {}).get(k) for k in ("corners", "sharpness", "banding_still")}, **res}
    entry["stages"] = SES.stages_of(entry)
    with open(LOG, "a") as f: f.write(json.dumps(entry) + "\n")
    if res.get("result_dir"): write_json(Path(res["result_dir"]) / "session.json", entry)
    with lock: S["result"] = res
    refresh_sessions()
    set_phase(phase, message, speak=speak)


def refresh_sessions():
    all_ = SES.load(LOG)
    with lock: S["history"] = [SES.summary_row(e) for e in all_[::-1][:50]]; S["today"] = SES.today(all_)


def used_auto(serial):
    d = read_json(AUTO_FILE, {})
    if serial not in d: d[serial] = datetime.now().astimezone().isoformat(timespec="seconds"); write_json(AUTO_FILE, d)


def auto_used(serial): return serial in read_json(AUTO_FILE, {})
def defective(serial): return read_json(DEFECTIVE, {}).get(serial)


def last_pass(cid):
    for h in S.get("history") or []:
        if h.get("camera") == cid and h.get("passed"): return h
    return None


def view():
    """The one place that decides what the page says about the camera: its id, what this station knows about it, and whether
    it may be unplugged. The page only displays this. Called with the lock held."""
    ph = S["phase"]; c = S.get("camera"); cid = ((c or {}).get("identity") or "").strip()
    day = lambda t: (t or "")[:16].replace("T", " ")
    if not c: calib = {"level": "free", "title": "No camera plugged in", "detail": ""}
    elif not cid: calib = {"level": "hold", "title": "This camera has no id", "detail": "Its sessions could not be told apart from another camera's. Set CAMERA_ID for the camera program."}
    elif defective(cid): calib = {"level": "hold", "title": "Marked defective", "detail": str(defective(cid).get("reason") or "")}
    elif last_pass(cid): calib = {"level": "free", "title": "Passed here before", "detail": f"Last pass {day(last_pass(cid).get('time'))}; result in {Path(last_pass(cid).get('result_dir') or '').name}."}
    elif any(h.get("camera") == cid for h in S.get("history") or []): calib = {"level": "free", "title": "Tried here, not passed yet", "detail": ""}
    else: calib = {"level": "free", "title": "New to this station", "detail": ""}
    if ph in ("recording_a", "recording_b"): un = {"level": "no", "icon": "✋", "title": "DO NOT UNPLUG", "why": "The arm is moving and recording."}
    elif ph in ("checking", "first_check", "solving", "qc"): un = {"level": "hold", "icon": "⏳", "title": "WAIT", "why": {"first_check": "Checking recording A.", "solving": "Working out the calibration.", "qc": "Banding check."}.get(ph, "Working. Wait for the result.")}
    elif ph == "passed": un = {"level": "go", "icon": "✔", "title": "UNPLUG NOW", "why": "Passed. The calibration is saved. Fit the next camera."}
    elif ph == "failed": un = {"level": "free", "icon": "✖", "title": "UNPLUG, OR TRY AGAIN", "why": "Failed. Set this camera aside, or reseat it and press Start."}
    elif ph == "no_camera" or not c: un = {"level": "free", "icon": "○", "title": "NO CAMERA", "why": "Fit a camera on the arm and plug it in."}
    else: un = {"level": "free", "icon": "○", "title": "OK TO UNPLUG", "why": "Nothing is running. Press Start to calibrate this camera."}
    # The stages of one session, in order, for the page's progress bar: done, now (with how far along), to do, or failed.
    run = [("recording_a", "Recording A", 90, "recording_a"), ("recording_b", "Recording B", 90, "recording_b"), ("first_check", "First check", 25, "first_check"),
           ("solving", "Solve", 45, "solving"), ("qc", "QC", 10, "qc")]
    tm = {**(S.get("timing") or {}), **(((S.get("result") or {}).get("timing") or {}) if ph in ("passed", "failed", "error", "stopped") else {})}
    since = round(time.time() - S["since"]); steps = []
    def step(label, state, seconds=None, typical=None):
        d = {"label": label, "state": state, "seconds": seconds}
        if state == "now" and typical: d["percent"] = min(97, round(100 * (seconds or 0) / typical))
        steps.append(d)
    step("Camera id", "done" if cid else ("now" if c else "todo"), None)
    step("Picture check", "now" if ph == "checking" else ("failed" if ph == "not_ready" else ("done" if S.get("check") else "todo")), since if ph == "checking" else None, 14)
    keys = [r[0] for r in run]; at = keys.index(ph) if ph in keys else None
    ended_bad = ph in ("failed", "error", "stopped"); reached = [r for r in run if tm.get(r[3]) is not None]
    for i, (key, label, typical, tkey) in enumerate(run):
        if at is not None: state = "done" if i < at else ("now" if i == at else "todo")
        elif ph == "passed": state = "done" if tm.get(tkey) is not None else "todo"
        elif ended_bad: state = "todo" if tm.get(tkey) is None else ("failed" if reached and reached[-1][0] == key else "done")
        else: state = "todo"
        step(label, state, since if state == "now" else tm.get(tkey), typical)
    return {"camera_id": cid or None, "calibration": calib, "unplug": un, "steps": steps}


def button_loop():
    """Start and Stop from push buttons read by the camera program. A Start press counts only while the screen says it can start."""
    base = None; lost_since = None
    while True:
        b = cfg().get("buttons", {})
        if not b.get("enabled"):
            with lock: S["buttons"] = {"enabled": False}
            base = None; time.sleep(1.0); continue
        try:
            d = cam_get("/buttons", 1.5)
            if not d.get("ok") or d.get("age_s") is None or d["age_s"] > 1.0: raise RuntimeError(d.get("error") or "button reader stalled")
        except Exception as e:
            lost_since = lost_since or time.time(); base = None
            with lock: S["buttons"] = {"enabled": True, "connected": False, "problem": str(e)[:120]}
            if S["phase"] in ("recording_a", "recording_b") and time.time() - lost_since > 2.5 and not want_stop.is_set():
                want_stop.set()                                         # the Stop button cannot be heard: do not keep moving
            time.sleep(0.3); continue
        lost_since = None; nc = bool(b.get("stop_normally_closed"))
        stop_held = (not d["stop_low"]) if nc else d["stop_low"]
        if base is None or base["boot"] != d["boot"]: base = d
        stop_edges = (d["stop_rises"] - base["stop_rises"]) if nc else (d["stop_falls"] - base["stop_falls"])
        if stop_edges > 0 or stop_held:
            if S["phase"] in BUSY or want_start.is_set() or auto["serial"]: want_stop.set()
            want_start.clear()
        elif d["start_falls"] - base["start_falls"] > 0:
            request_start("START button pressed")
        elif d.get("power_taps", 0) - base.get("power_taps", 0) > 0:      # a Pi's own button: one button, so it toggles; during a countdown it cancels
            if S["phase"] in BUSY or auto["serial"]: want_stop.set(); want_start.clear()
            else: request_start("Pi button pressed")
        base = d
        with lock: S["buttons"] = {"enabled": True, "connected": True, "start_down": d["start_low"], "stop_down": stop_held,
                                   "pi_button": bool(d.get("power_ok")), "pi_button_down": bool(d.get("power_down"))}
        time.sleep(0.08)


NO_CAMERA = "No camera. Fit the next camera on the arm and plug it in."
AUTO_WAIT = 8                     # seconds between Ready and the automatic start
UNPLUG_S = 3                      # a camera counts as newly plugged in only after its device was gone this long
auto = {"serial": None, "at": 0.0}
start_req = {"source": "Start pressed"}


def request_start(source):
    with lock: ok = S["phase"] in CAN_START and S["camera"] is not None
    if ok: start_req["source"] = source; want_stop.clear(); want_start.set()
    return ok


def loop():
    """Automatic start: once per camera, ever (kept in station/auto_started.json across restarts), only for a camera that was
    really plugged in (its device absent at least UNPLUG_S seconds before), after a countdown that any Stop cancels, and never
    for a camera marked defective or one that already passed here. A missing picture or a slow answer from the camera program
    is not an unplug: it never wipes a verdict and never arms an automatic start. Any start uses up the automatic start."""
    last_serial = None; recheck = 0.0; gone_since = None; plugged_fresh = False
    while True:
        try:
            state, c = camera(); phase = S["phase"]
            if state == "gone":
                gone_since = gone_since or time.time(); auto["serial"] = None
                if time.time() - gone_since >= UNPLUG_S: plugged_fresh = True
                if phase in ("passed", "failed"):          # unplugged after its verdict: keep the verdict on show until the next camera is fitted
                    last_serial = None; want_start.clear(); time.sleep(0.5); continue
                if phase != "no_camera" or S["message"] != NO_CAMERA:
                    set_phase("no_camera", NO_CAMERA, camera=None, check=None, exposure_us=None)
                last_serial = None; want_start.clear(); time.sleep(1.0); continue
            gone_since = None
            if state == "no_picture":                      # a device is there but no picture (or the camera program is slow): show it, change nothing else
                if phase in ("ready", "checking", "not_ready", "no_camera"):
                    with lock: S["message"] = "The camera is plugged in but its picture is not arriving. Waiting for it."
                time.sleep(1.0); continue
            serial = (c.get("identity") or "").strip()
            if not serial:
                if phase != "not_ready" or time.time() > recheck:
                    set_phase("not_ready", "This camera reports no id, so its sessions could not be told apart from another camera's. "
                              "Set CAMERA_ID for the camera program.", camera={**c, "serial": "no id"}, check=None, exposure_us=None)
                    recheck = time.time() + 6
                last_serial = None; want_start.clear(); time.sleep(1.0); continue
            if serial != last_serial or phase == "no_camera" or (phase == "not_ready" and time.time() > recheck):
                first = serial != last_serial or phase == "no_camera"; last_serial = serial
                fresh_plug = plugged_fresh
                set_phase("checking", f"Camera {serial} found. Checking the picture.", speak="Camera found." if first else None, camera={**c, "serial": serial}, result=None if first else S["result"], timing={} if first else S.get("timing"))
                time.sleep(1.0); ok, det = check_camera(c)
                with lock: S.update(check=det, exposure_us=det.get("exposure_us"))
                if ok:
                    set_phase("ready", "Ready. Check the camera is seated and your hands are clear, then press Start." + (" Note: " + det["warning"] if det.get("warning") else ""), speak="Ready." if first else None)
                else:
                    set_phase("not_ready", det["problem"], speak="Not ready." if first else None); recheck = time.time() + 6
                want_start.clear()
                bad = defective(serial); passed = last_pass(serial)
                arm = ok and fresh_plug and not auto_used(serial) and not bad and not passed and cfg().get("auto_start", True)
                if ok and bad:
                    set_phase("ready", f"This camera is marked defective: {bad.get('reason')} It was not started automatically. Press Start to run it anyway.")
                elif ok and passed and fresh_plug and not auto_used(serial):
                    set_phase("ready", "This camera already passed at this station. It was not started automatically. Press Start to run it again.")
                if ok: plugged_fresh = False                # a newly plugged camera keeps its automatic start through failed picture checks
                auto["serial"] = serial if arm else None; auto["at"] = time.time() + AUTO_WAIT
                continue
            if auto["serial"] == serial and phase == "ready":
                if want_stop.is_set():
                    auto["serial"] = None; used_auto(serial); want_stop.clear()
                    set_phase("ready", "Automatic start cancelled. Press Start when you want to calibrate this camera.")
                elif time.time() >= auto["at"]:
                    auto["serial"] = None; request_start("automatic start after the countdown (camera newly plugged in)")
                else:
                    left = max(0, int(auto["at"] - time.time()) + 1)
                    with lock: S["message"] = f"Camera ready. Starting by itself in {left} s: take your hands off the arm. Press Stop to cancel."
            if want_start.is_set():
                want_start.clear(); auto["serial"] = None
                if phase in CAN_START:
                    calibrate(serial, start_req["source"])
            time.sleep(0.3)
        except Exception as e:
            set_phase("error", f"Station problem: {type(e).__name__}: {str(e)[:160]}"); time.sleep(3)


def lease_loop():
    """Tell the camera program every 30 s that this station still owns it."""
    while True:
        try: S["camera_owner"] = cam_post("/lease", timeout=5)
        except urllib.error.HTTPError as e:
            try: S["camera_owner"] = json.loads(e.read())
            except Exception: S["camera_owner"] = {"ok": False, "error": f"HTTP {e.code}"}
        except Exception as e: S["camera_owner"] = {"ok": False, "error": type(e).__name__}
        time.sleep(30)


class Hd(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def send(self, code, body):
        b = json.dumps(body).encode(); self.send_response(code); self.send_header("Content-Type", "application/json"); self.send_header("Content-Length", str(len(b)))
        self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(b)
    def do_GET(self):
        if self.path.split("?")[0] == "/state":
            with lock: body = {**S, "view": view(), "seconds_in_phase": round(time.time() - S["since"]), "run_seconds": round(time.time() - S["run_started"]) if S.get("run_started") and S["phase"] in BUSY else None, "config": cfg(),
                               "can_start": S["phase"] in CAN_START and S["camera"] is not None,
                               "moving": S["phase"] in ("recording_a", "recording_b"), "station_id": STATION_ID, "auto_countdown": auto["serial"] is not None}
            return self.send(200, body)
        self.send(404, {"error": "not found"})
    def do_POST(self):
        p = self.path.split("?")[0]
        if p == "/start":
            ok = request_start("Start pressed on the station page")
            return self.send(200, {"ok": ok, "why": "" if ok else "The station is not ready to start: " + S["message"]})
        if p == "/stop":
            want_stop.set(); want_start.clear()        # the sweep loop sends the one stop signal; never signal twice
            return self.send(200, {"ok": True})
        if p == "/sethome":
            if S["phase"] in BUSY: return self.send(200, {"ok": False, "why": "The station is busy."})
            out = subprocess.run([PY, str(IN / "so101-white-registers-readonly.py")], capture_output=True, text=True, cwd=str(H.ARM_CWD), timeout=30).stdout
            row = next((l.split()[1:6] for l in out.splitlines() if l.startswith("Present_Position")), None)
            if not row or len(row) != 5: return self.send(200, {"ok": False, "why": "Could not read the arm."})
            c = read_json(CONFIG, {}); c["home"] = [int(v) for v in row]; write_json(CONFIG, c)
            return self.send(200, {"ok": True, "home": c["home"]})
        self.send(404, {"error": "not found"})


def shutdown(signum, _frame):
    """The station is being stopped: end a running sweep first, so the arm never keeps moving without the station watching it."""
    p = proc.get("p")
    if p is not None and p.poll() is None:
        want_stop.set(); p.send_signal(signal.SIGTERM)
        try: p.wait(timeout=15)
        except subprocess.TimeoutExpired: pass
    os._exit(0)


if __name__ == "__main__":
    cfg(); refresh_sessions()
    signal.signal(signal.SIGTERM, shutdown); signal.signal(signal.SIGINT, shutdown)
    for target in (loop, button_loop, lease_loop):
        threading.Thread(target=target, daemon=True).start()
    print(f"station {STATION_ID} on {H.STATION_URL}/state, camera program {CAM}, data in {DATA}", flush=True)
    ThreadingHTTPServer(("127.0.0.1", H.STATION_PORT), Hd).serve_forever()
