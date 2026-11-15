#!/usr/bin/env python3
"""One recording while the arm sweeps, with everything kept so it can be studied afterwards.

Each call makes a folder <data>/runs/<time>-<plan>-<label>/ holding:
  recording/         the camera program's recording: video.mjpeg (every camera frame, untouched), frames.csv (one line per
                     frame: time, place in the file, exposure), device.json (the camera and the recording's own check)
  controller.json    the arm controller's own evidence file: every 0.1 s the goal sent and the pose read back
  sweep-code.py      the exact sweep program that ran      plan.json   the plan it ran
  run.json           the command, plan, times, and whether torque was on
  console.log        everything the arm program printed

  python run_sweep.py --plan gentle --seconds 60 --speed 120 --travel 150                         dry run: arm does not move
  python run_sweep.py --plan-file plans/close2/plan.json ... --enable-torque --preflight record.json --label c123456A

This program sends nothing to a motor itself. Motion goes only through arm/so101-five-joint-sweep.py, which keeps the
teleoperation program's preflight, limits, lag abort, watchdog and verified torque-off.
"""
import json, os, shutil, signal, subprocess, sys, tarfile, time, urllib.request
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hostconf as H                                                           # noqa: E402

SWEEP = H.ARM_DIR / "so101-five-joint-sweep.py"
CAM = H.CAMERA_URL
LOCAL_REC = os.environ.get("STATION_LOCAL_REC")                                # the camera program's recording folder, when on this machine


def cam_post(path):
    req = urllib.request.Request(CAM + path, method="POST", headers={"X-Station": os.environ.get("STATION_ID", "unnamed")})
    return json.loads(urllib.request.urlopen(req, timeout=15).read())


def cam_status():
    return json.loads(urllib.request.urlopen(CAM + "/status", timeout=5).read())


def fetch_recording(name, folder):
    """Copy a finished recording from the camera program into folder/recording."""
    out = folder / "recording"; out.mkdir(parents=True, exist_ok=True); total = 0
    if LOCAL_REC:                                   # same machine: link the files instead of copying them
        for f in sorted((Path(LOCAL_REC).expanduser() / name).iterdir()):
            if not f.is_file(): continue
            try: os.link(f, out / f.name)
            except OSError: shutil.copy(f, out / f.name)
            total += f.stat().st_size
        return total
    resp = urllib.request.urlopen(f"{CAM}/record/tar?name={name}", timeout=60)
    with tarfile.open(fileobj=resp, mode="r|") as tar:
        for m in tar:
            if not m.isfile(): continue
            with open(out / Path(m.name).name, "wb") as f:
                src = tar.extractfile(m)
                while True:
                    b = src.read(1 << 20)
                    if not b: break
                    f.write(b); total += len(b)
    return total


def attempt(argv):
    label = "dry"
    if "--label" in argv:
        i = argv.index("--label"); label = argv[i + 1]; del argv[i:i + 2]
    armed = "--enable-torque" in argv
    if armed and not (label != "dry" and label.replace("-", "").isalnum() and len(label) <= 12):
        sys.exit("--label <short name> is needed for a powered recording")
    plan = argv[argv.index("--plan") + 1] if "--plan" in argv else "gentle"
    if "--plan-file" in argv:
        plan = Path(argv[argv.index("--plan-file") + 1]).parent.name
    (H.DATA / "runs").mkdir(parents=True, exist_ok=True)
    folder = H.DATA / "runs" / f"{datetime.now():%Y%m%d-%H%M%S}-{plan}-{label}"
    folder.mkdir(parents=True)
    shutil.copy(SWEEP, folder / "sweep-code.py")
    if "--plan-file" in argv: shutil.copy(argv[argv.index("--plan-file") + 1], folder / "plan.json")
    before = set(H.ARM_DIR.glob("so101-teleop-*.json"))
    secs = float(argv[argv.index("--seconds") + 1]) if "--seconds" in argv else 10
    started_rec = cam_post(f"/record/start?name={folder.name}&seconds={int(secs + 30)}")
    if not started_rec.get("ok"): sys.exit(f"The camera recording did not start: {started_rec}")
    t_wait = time.time()
    while time.time() - t_wait < 12:                # the arm moves only once frames are arriving in the recording
        if cam_status()["record"]["frames"] > 0: break
        time.sleep(0.3)
    else: sys.exit("The camera recording did not start delivering frames")
    started = time.time()
    with open(folder / "console.log", "w") as log:
        p = subprocess.Popen([H.PY, str(SWEEP), *argv], cwd=str(H.ARM_CWD), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        def pass_on(*_):                             # once only: a second stop must never reach the arm program while it releases torque
            signal.signal(signal.SIGTERM, signal.SIG_IGN); p.terminate()
        signal.signal(signal.SIGTERM, pass_on)
        for line in p.stdout:
            sys.stdout.write(line); log.write(line); log.flush()
        code = p.wait()
    t0 = time.time()
    stop = cam_post("/record/stop")
    res = stop.get("result") or cam_status()["record"].get("result") or {}
    size = fetch_recording(folder.name, folder)
    print(f"Camera recorded {res.get('frames')} frames ({'verified' if res.get('ok') else 'NOT verified: ' + str(res.get('verdict'))[:120]}); "
          f"fetched {size / 1e6:.0f} MB in {time.time() - t0:.0f} s")
    if not res.get("ok") and code == 0: code = 4
    new = sorted(set(H.ARM_DIR.glob("so101-teleop-*.json")) - before)
    if new: shutil.copy(new[-1], folder / "controller.json")
    (folder / "run.json").write_text(json.dumps({
        "plan": plan, "label": label, "armed": armed, "command": argv, "exit_code": code,
        "started_unix": started, "ended_unix": time.time(), "bytes": size,
        "controller_evidence": str(new[-1]) if new else None}, indent=1))
    print(f"\nSaved: {folder}  (controller log {'kept' if new else 'MISSING'}, exit code {code})")
    return code


if __name__ == "__main__":
    sys.exit(attempt(sys.argv[1:]))
