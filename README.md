# SO-101 stereo camera calibration station

An SO-101 robot arm holds a USB stereo camera and sweeps it in front of a printed ChArUco board. The camera records two
sweeps, A and B; the station calibrates both lenses and the pair from them with plain OpenCV, checks the result against
the sweep it was not fitted on, and gives a pass or fail. An operator fits a camera, presses Start (or just plugs the
camera in), waits about four minutes, reads the verdict and swaps the camera.

Built and tested with the [OpenRobot stereo camera](https://theopenrobot.com) (two 1920 x 1200 lenses, side by side in
one 3840 x 1200 picture). Another USB stereo camera that sends both lenses side by side as MJPEG should also work; see
"What you need".

> **Safety.** The arm moves by itself during a sweep. Keep people out of its reach while it moves, and keep the arm's
> power plug within reach: the software limits are not a hardware stop.

`ARCHITECTURE.md` is the canonical shape: which program owns what, and the one path a camera takes.

## What you need

- An SO-101 follower arm (the "white" arm in the code) with its USB bus servo board, calibrated with LeRobot. The leader
  arm (the "black" arm) is only needed for teleoperation (`arm/so101-five-joint-teleop.py`); a sweep never uses it.
- A camera mount on the follower's wrist, so the camera looks out along the gripper. For the OpenRobot camera, print
  `hardware/so101-wrist-mount.stl` (see "The wrist mount" below).
- A USB stereo camera that delivers MJPEG with both lenses side by side in one frame (left half = left lens), e.g.
  3840 x 1200 for two 1920 x 1200 lenses. The station was built with the OpenRobot stereo camera. Fixed exposure through the standard V4L2 exposure controls helps a lot.
- The ChArUco board: 12 x 9 squares of 30 mm, ArUco markers of 22 mm, dictionary DICT_5X5_1000 (88 inner corners),
  printed flat and stuck to something rigid, standing upright in front of the arm, evenly lit, without glare.
- A Mac or Linux computer for the station (it drives the arm over USB), and a Linux machine for the camera program (a
  Raspberry Pi 5 works; the station computer itself if it runs Linux).
- Python 3.10+ with `lerobot` (for the arm programs), `opencv-python` (4.7 or later, for `cv2.aruco.CharucoDetector`)
  and `numpy`; `ffmpeg` and `ffprobe`; on the camera machine also `v4l-utils` (`v4l2-ctl`).
- Optional, on a Pi: two push buttons (Start, Stop) on the pin header (see `camera/stereo_stream.py`), and the Pi's two
  board lights as a status light.

## The wrist mount

`hardware/so101-wrist-mount.stl` replaces the gripper on the follower arm and holds one OpenRobot stereo camera module.

1. Take the gripper (motor 6 and its jaw) off the wrist-roll motor (motor 5). Keep its four screws.
2. Screw the mount onto the wrist-roll motor's horn with those four screws, where the gripper was.
3. Slide the camera module's cap clip onto the mount's tongue, and put the module's own bolt through the slot in the
   tongue. Swapping cameras is then one bolt.

We printed it in PETG with 0.2 mm layers and a little support under the bar; it weighs about 13 g. The calibration
uses only the camera's pictures, never the arm's position, so the mount only has to hold the camera firmly: a tight bolt
is enough. For another camera, design a mount that holds it just as rigidly.

## Layout

```
station/      the station: state machine, operator page, solver, checks, arm sweep plans
  calib_station.py      the session state machine (port 18772)
  calib_dashboard.py    the operator page (port 18771) + calib_console.html, fonts/
  stereo_solver.py      the open solver (OpenCV ChArUco calibration), runnable on its own
  board.py              the board definition and detector; prints the board
  sessions.py           what a session is, and every count on the page
  pick_exposure.py      picture check: picks a fixed exposure, checks the board is seen by both lenses
  qc_banding.py         quality check for horizontal banding (stripes along the sensor rows)
  run_sweep.py          one recording while the arm sweeps   make_preflight_waived.py  the preflight record it needs
  hostconf.py           settings for this machine (station.env)
  plans/                arm sweep plans (close2 is the default)
arm/          SO-101 programs: sweep, teleoperation, preview, torque-off calibration, read-only register readers
camera/       stereo_stream.py: the camera program (Linux, V4L2 + ffmpeg)
hardware/     so101-wrist-mount.stl: the printed wrist mount for the OpenRobot camera
run.sh        start / stop / check
```

Where things run:

| Program | Machine | Port |
|---|---|---|
| `camera/stereo_stream.py` | Linux machine with the camera plugged in (a Pi 5, or the station machine) | 8090 |
| `station/calib_station.py` | the machine the arm's USB is plugged into (Mac or Linux) | 18772 (local only) |
| `station/calib_dashboard.py` | same machine as the station | 18771 |

On one Linux machine everything runs together. With a Mac, plug the arm into the Mac and the camera into a Pi, and set
`STATION_CAMERA_URL=http://<pi>:8090` in `station.env` on the Mac.

## Set up

1. Print the board: `python station/board.py --print board.png`, print it at a scale where one square measures exactly
   30 mm (measure it; if it is not 30 mm, put the measured size in `STATION_BOARD`), and mount it flat.
2. Calibrate the follower arm's servos with LeRobot (or `python arm/so101-calibrate-safe.py follower`, which only writes
   calibration registers with torque off), then copy the calibration file to `arm/profile/calibration.json` or point
   `SO101_PROFILE` at it. `arm/profile/calibration.example.json` shows the format; it belongs to another arm, do not use it.
3. Tell the station which USB board is the follower: `python arm/so101_host.py` lists the USB serial devices it sees.
   With one board plugged in it is found by its USB id; otherwise copy `arm/arm.example.json` to `arm/arm.json` and fill
   in `follower_serial` (and `leader_serial` for teleoperation).
4. Copy `station.env.example` to `station.env` and set what differs on this machine (at least `STATION_PY` and
   `STATION_DATA`). `./run.sh check` shows what each setting resolves to; it starts nothing.
5. On the camera machine: `./run.sh camera` (or `python3 camera/stereo_stream.py`), then open `http://<it>:8090/` and
   check the live picture. The camera id the station uses is `CAMERA_ID`, else the camera's USB serial; if all your
   cameras report the same USB serial, set `CAMERA_ID` before each camera.
6. Start the station: `./run.sh start`, open `http://127.0.0.1:18771/`.
7. Set the home pose: with the arm's torque off, move it by hand so the camera faces the middle of the board from about
   0.45 m, and press SET HOME on the page. Every sweep starts and ends there.
8. Try the plan without torque first: `SWEEP_AIM_CAP=100 python station/run_sweep.py --plan-file station/plans/close2/plan.json --home
   <five ticks> --seconds 74 --speed 400 --travel 2100` runs the sweep loop against the arm and logs the goals it would
   send, without writing to a motor (it still makes a recording). The plans were designed for one bench layout; if
   the board leaves the picture, change the plan or the home pose.

Settings that are not about the machine live in `<data>/station/config.json` (read on every use; the defaults are at
the top of `calib_station.py`): `plan`, `home`, `speed`, `travel`, `torque_limit`, `aim_cap_deg`, `exposure_fixed_us`,
`still_banding_limit`, `require_eye_size`, `auto_start`, `buttons`.

## Running it

Fit a camera on the arm and plug it in. The station reads the camera's id, picks an exposure (a full sweep of
exposures every few hours, or when the light on the board changes by a fifth), checks that both lenses see the board,
and scores banding on still pictures. Then press Start; a camera that was just plugged in also starts by itself after
an 8 s countdown, once per camera (any Stop cancels it). The arm records A, then B; then, with the arm stopped, the
solver checks A, solves the pair, and the banding check runs on A. The page says PASS or FAIL and why, and whether the
camera may be unplugged. Stop on the page, the Escape key, or the STOP button ends a sweep at once; the arm program then
releases torque and reads it back.

Each session leaves, in the data folder: `runs/<time>-<plan>-<camera><A|B>/` (the recording, the arm's evidence file,
the exact sweep program and plan, the console log), `station/results/<camera>-<time>/` (`calibration.yaml`,
`result.json`, the first check's result, the solver's logs, `session.json`) and one line in `station/log.jsonl`.

To mark a camera defective (never started automatically): add it to `<data>/station/defective.json` as
`{"<camera id>": {"reason": "..."}}`. To run a camera despite a still-picture banding fail: add its id to
`<data>/station/qc_override.json` (a JSON list).

The solver also runs on its own, on any two recordings:

```
python station/stereo_solver.py check <recording A> --out <folder>
python station/stereo_solver.py solve <recording A> <recording B> --out <folder>
```

## What the checks mean

The calibration is fitted on recording A only. Recording B is a second, separate sweep, held out to test it.

| Check | Limit | What it guards against |
|---|---|---|
| distinct still views per lens (A) | at least 25 | Only frames in which the board's corners move less than 1 px during the exposure are used (blur and rolling shutter spoil corners), and near-identical views count once. Too few: the fit is underdetermined. |
| tilted views per lens (A) | at least 5 views turned 20 degrees or more | A planar board seen only face-on cannot separate focal length from distance. |
| lens sharpness, left against right (A) | the sharper lens at most 2x the blurrier one | Both lenses look at the same board, so their board-area sharpness (variance of the Laplacian inside the board's corners) should match. A lens far blurrier than its partner is dirty, filmed or out of focus. Corners are still found on it, and its error can stay under the limit, so this is checked directly. |
| picture reached per lens (A) | at least 12 of 16 cells | Distortion is strongest at the edges; where the board never went, the lens model is a guess. |
| fit error per lens (A) | RMS at most 1.0 px | The model does not even describe the views it was fitted on. |
| held-out error per lens (B) | median at most 0.5 px | Each B view's board pose is found with A's lens model fixed, and the corners projected back. Overfitting and a bad model show up here. |
| views with the board in both lenses | at least 15 in A and in B | The stereo fit and the row check need the same corners seen by both lenses. |
| rows lining up (B) | median at most 0.5 px | After rectification with A's stereo model, the same corner must land on the same row in both lenses; stereo matching searches along that row. |
| rectified view (A) | **110 x 80 degrees, plus or minus 2 degrees**: fails below 108 x 78 | After both lenses are straightened onto shared rows and the black borders cropped, this much picture must remain. 110 x 80 is the camera's spec. The view shrinks when the lenses point apart or are out of line. A view wider than 112 x 82 is reported in `result.json` but does not fail: a wider view is not a defect. For other lenses, set `STATION_RECT_FOV` to the spec (for example `90x60`) and `STATION_RECT_FOV_TOL` to the tolerance; `0x0` switches the check off. |
| A against B | right-lens position differs at most 2.5 mm | The pair is fitted again on B alone (lenses fixed); a camera is rigid, so the two answers must agree. A difference means the camera or the board moved, or the result depended on the frames. |
| banding (QC) | 0.6 on still pictures, 1.0 on recording A | Faint stripes along the sensor rows: a camera fault, not a calibration problem. Both limits are provisional. |

The lens model is OpenCV's rational model (k1 k2 p1 p2 k3 k4 k5 k6), fitted with `cv2.calibrateCamera`; the pair with
`cv2.stereoCalibrate` with both lens models fixed, and `cv2.stereoRectify` (alpha 0). The limits are at the top of
`stereo_solver.py`. `result.json` also reports the baseline, the rectified field of view, each lens's distance range and
tilt, and the time each step took.

## What it does not do

- It does not write the calibration into the camera, and does not upload anything: results stay in the data folder.
- It does not calibrate a camera's motion sensor (IMU) against the lenses, nor the time offset between them.
- It does not check that people are out of the arm's reach. The arm programs refuse to move without a preflight record,
  and the station writes one that says the operator keeps people clear: pressing Start is that statement. Keep people
  out of the follower's reach while it moves; the software limits are not a hardware stop, so keep the arm's power plug
  within reach.
- It does not know your bench: the sweep plans and the default home pose need checking against your arm and board.
- It has no fisheye model; very wide lenses (beyond about 120 degrees) may need `cv2.fisheye` instead of the rational model.
- It has been run only with one camera model at one bench; the camera program's V4L2 recording path has not been run
  on hardware yet.

## Verification

Changes are checked on real recordings and real runs, not with unit tests or simulated hardware. The arm programs keep
their own offline self-test (`python arm/so101-five-joint-sweep.py --self-test`, opens no hardware).
