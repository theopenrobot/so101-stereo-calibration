# Calibration station: architecture

This is the canonical shape of the station. A change goes in only if it fits this shape; a change that needs a different
shape changes this file first, in the same edit.

## Programs and what each one owns

| Program | Runs on | Owns | Must not |
|---|---|---|---|
| `camera/stereo_stream.py` (port 8090) | the Linux machine the camera is plugged into | the camera: live view, recordings, exposure, a Pi's lights and buttons | take a command (POST) without an `X-Station` header, or from a second station while one owns it; change exposure while recording |
| `station/calib_station.py` (port 18772, local only) | the machine the arm is plugged into | the session state machine, the arm (only through `run_sweep.py` and `arm/so101-five-joint-sweep.py`), the solver runs, the session log | run more than one copy against the same camera program; start a run except by Start, a button, or the one automatic start |
| `station/stereo_solver.py` | started by the station, with the arm stopped | turning recordings into a calibration and a verdict | read anything but the two recordings; write anywhere but its result folder |
| `station/calib_dashboard.py` + `calib_console.html` (port 18771) | same machine as the station | the operator page | count or parse sessions itself (it calls `sessions.py`); move the arm (it only forwards Start, Stop and Set home) |
| `station/sessions.py` | (library) | what a session is, its stages, and every count shown on a screen | be bypassed: nothing else reads `station/log.jsonl` |
| `arm/so101-five-joint-sweep.py` (on `so101-five-joint-teleop.py`) | started by `run_sweep.py` | every motor write: preflight record, torque limit, goal rate limit, travel limit, aim cap, joint limits, lag abort, health checks, watchdog, return to home, torque-off readback, evidence file | move without a fresh preflight record for the exact command; start further from home than the plan itself goes |

Only one station runs at a time.

## The one path for a camera

1. Camera plugged in (its device appears after being absent at least 3 s) or Start pressed.
2. Picture check: camera id read, exposure picked (full sweep every few hours or when the light on the board changes by a
   fifth; else the saved value), board seen in both lenses, banding scored on still pictures.
3. Exposure set again on the camera, then recording A, then recording B. The arm moves only here.
4. Each recording's exposure is read back from its `frames.csv` when the camera program wrote one; a recording made at
   another exposure is not used.
5. With the arm stopped (sequential, nothing runs beside the arm): the solver's first check on A alone. If A fails it
   (too few distinct or tilted views, the board did not reach enough of the picture, the fit is poor), the session ends
   (fail fast): B cannot change that.
6. The solve: lens models and stereo model from A; held-out error, row offset and repeat measured on B.
7. Banding check on recording A. A check that could not run is a fail.
8. The session is logged once (`finish`). The calibration file, the solver's result and the recordings stay in the data
   folder. Nothing is written to the camera and nothing is sent anywhere.

There is no other path.

## Automatic start

Once per camera, ever (`station/auto_started.json`, kept across restarts). Only for a camera that was really unplugged and
plugged in, and never for one marked defective (`station/defective.json`) or one that already passed here. Any start, by
hand or automatic, uses it up. Any Stop (page, STOP button, a Pi's own button) cancels the countdown. A missing picture or
a slow answer from the camera program is not an unplug: it never wipes a verdict and never arms an automatic start.

## Safety

- The arm moves only between Start and the end of recording B, and only through the sweep program, which keeps every
  limit of the teleoperation program and refuses to start without a fresh preflight record for the exact command.
- Stop ends a sweep at once (one stop signal only: a second one must never interrupt the torque release); the arm program
  then releases torque and reads it back. If the readback does not confirm torque off, the page and the voice say to pull
  the arm's power plug.
- A lost camera picture (three checks a second apart) or, with buttons enabled, a lost Stop button stops the sweep.
- Stopping the station program ends a running sweep first (torque released), then exits.
- A sweep starts only near home: further than the usual limit is accepted only when every joint is somewhere the plan
  itself goes (e.g. after a Stop mid-sweep), and the arm then eases back to home first.

## State and settings

- The data (`runs/`, `station/`, `preflight-run.json`) lives in the folder `STATION_DATA` names, outside the repo. The
  paths below are inside it. Machine settings come from `station.env` (see `station/hostconf.py`).
- `station/config.json`: read on each use, written only in one step (`write_json`). If it cannot be read the last good
  settings stay in use and the page shows the problem; the defaults never replace it silently.
- `station/log.jsonl`: one line per session; read only through `sessions.py`.
- `station/exposure.json`: the exposure picked by the last full sweep, with the board brightness it was picked at.
- `station/results/<camera>-<time>/`: `calibration.yaml`, `result.json`, `session.json`, the solver's logs and its
  detections cache.

## Verification

Changes are verified on real recordings and real runs, with their logs. There are no unit tests, no replay or
fake-hardware tests and no CI. Report what was run and what was not.
