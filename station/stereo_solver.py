#!/usr/bin/env python3
"""Stereo calibration from two recordings of the ChArUco board, with plain OpenCV.

    python stereo_solver.py check <recording A> --out <folder>
    python stereo_solver.py solve <recording A> <recording B> --out <folder>

A recording is a run folder (the video is found in it: recording/video.mjpeg as the camera program writes it, or
video.hevc, video.mkv ...) or a video file; frames.csv next to the video, when present, gives the frame rate and exposure.
Each frame holds both lenses side by side: left half = left lens (cam0), right half = right lens (cam1).

What it does, in order:
  1. Decode every second frame (ffmpeg) and find the board's inner corners in each lens (cv2.aruco.CharucoDetector).
  2. Keep still frames only: the board's corners may move at most MAX_BLUR_PX during one exposure (motion blur and
     rolling shutter both spoil a corner), measured from the corner motion between neighbouring decoded frames.
  3. Keep distinct views only: a view is dropped when an already kept view saw the same corners within DISTINCT_PX.
  4. Per lens, on recording A: cv2.calibrateCamera with the rational distortion model (k1 k2 p1 p2 k3 k4 k5 k6);
     views fitting much worse than the rest are dropped once and the fit repeated.
  5. On recording A, with both lens models fixed: cv2.stereoCalibrate (rotation R and translation T from the left lens
     to the right lens), then cv2.stereoRectify.
  6. Recording B is held out: it never enters a fit. Per lens, each B view's board pose is found (solvePnP) with A's
     lens model and the corners are projected back: the held-out reprojection error. With A's stereo model, the same
     board corner seen by both lenses must land on the same row after rectification: the held-out row offset. B is
     also fitted alone with stereoCalibrate (lens models fixed) and its T compared with A's: the repeat check.

`check` runs steps 1 to 4 on A and the checks that need only A (enough distinct views, enough tilted views, the board
reached enough of the picture); the station runs it first and stops when A already fails (B cannot change that).
`solve` runs everything (reusing check's work on A when it is in the same folder) and writes:
  calibration.yaml   OpenCV FileStorage: both lens models, R, T, E, F, R1, R2, P1, P2, Q, the picture size
  result.json        every number below, every check with its limit, and the verdict
The last line printed is the result as one JSON line. Progress goes to stderr. Nothing is uploaded or written anywhere
else, and nothing is written to the camera.
"""
import argparse
import csv
import json
import os
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import board as B                                                      # noqa: E402

STEP = int(os.environ.get("SOLVER_STEP", "2"))        # decode every STEP-th frame
MIN_CORNERS = 12                                      # corners a lens must see in a view for it to be used
MAX_BLUR_PX = 1.0                                     # most a corner may travel during one exposure
DISTINCT_PX = 25.0                                    # views closer than this (median over shared corners) count as one
MAX_VIEWS = 80                                        # views per lens given to calibrateCamera (and pairs to stereoCalibrate)
TILT_DEG = 20.0                                       # a view is "tilted" when the board plane is turned this far from the lens
GRID = 4                                              # the picture is split GRID x GRID for the coverage check
CELL_MIN = 8                                          # corner sightings that make a cell count as reached
FLAGS = cv2.CALIB_RATIONAL_MODEL
MODEL = "OpenCV rational: k1 k2 p1 p2 k3 k4 k5 k6"
LIMITS = {                                            # change them here; each check below says what it guards against
    "views": 25, "tilted": 5, "cells": 12, "fit_rms_px": 1.0, "pairs": 15,
    "holdout_median_px": 0.5, "rect_dy_median_px": 0.5, "repeat_mm": 2.5,
}
TERM = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 100, 1e-7)


def log(*a):
    print(*a, file=sys.stderr, flush=True)


# ---------------------------------------------------------------- reading a recording

def find_video(path):
    p = Path(path)
    if p.is_file():
        return p
    for name in ("recording/video.mjpeg", "recording/video.hevc", "video.mjpeg", "video.hevc", "video.h265", "h265/video.hevc", "video.mkv", "video.mp4"):
        if (p / name).is_file():
            return p / name
    raise FileNotFoundError(f"no video found in {p}")


def probe(video):
    out = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=width,height",
                          "-of", "json", str(video)], capture_output=True, text=True, check=True).stdout
    s = json.loads(out)["streams"][0]
    return int(s["width"]), int(s["height"])


def recording_timing(video):
    """(frames per second, exposure in seconds) from the recording's frames.csv when it has one, else (30, 1/30)."""
    for f in (video.parent / "frames.csv", video.parent.parent / "frames.csv"):
        try:
            with open(f) as fh:
                rows = list(csv.DictReader(fh))
        except OSError:
            continue
        fps, expo = 30.0, None
        if len(rows) > 30 and rows[0].get("host_ns"):
            t = [int(r["host_ns"]) for r in rows]
            fps = (len(t) - 1) / ((t[-1] - t[0]) / 1e9)
        e = [int(r["exposure_us"]) for r in rows[15:] if (r.get("exposure_us") or "").isdigit()]
        if e:
            expo = float(np.median(e)) / 1e6
        return fps, expo or 1.0 / fps
    return 30.0, 1.0 / 30.0


def frames(video, w, h):
    """Yield (frame number, grey picture) for every STEP-th frame."""
    hw = os.environ.get("DECODE_HWACCEL") or ("videotoolbox" if sys.platform == "darwin" else "")
    for attempt in ([*(["-hwaccel", hw] if hw else [])], []):
        cmd = ["ffmpeg", "-v", "error", "-nostdin", *attempt, "-i", str(video), "-vf", f"select=not(mod(n\\,{STEP}))",
               "-fps_mode", "passthrough", "-f", "rawvideo", "-pix_fmt", "gray", "-"]
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        n = 0
        try:
            while True:
                raw = p.stdout.read(w * h)
                if len(raw) < w * h:
                    break
                yield n * STEP, np.frombuffer(raw, np.uint8).reshape(h, w)
                n += 1
        finally:
            p.stdout.close(); p.kill(); p.wait()
        if n:
            return
    raise RuntimeError(f"ffmpeg decoded no frames from {video}: {p.stderr.read().decode(errors='replace')[-300:]}")


_local = threading.local()


def _detect(item):
    n, g = item
    det = getattr(_local, "det", None)
    if det is None:
        det = _local.det = B.make_detector()
    w = g.shape[1] // 2
    return n, [B.detect(det, g[:, :w], MIN_CORNERS), B.detect(det, g[:, w:], MIN_CORNERS)]


def detect_recording(path):
    """{"size": (w, h) per lens, "fps", "exposure_s", "frames": {n: [left or None, right or None]}}."""
    video = find_video(path)
    w, h = probe(video)
    fps, expo = recording_timing(video)
    out = {}
    with ThreadPoolExecutor(max_workers=max(2, (os.cpu_count() or 4) - 1)) as ex:
        pending = []
        for item in frames(video, w, h):
            pending.append(ex.submit(_detect, (item[0], item[1].copy())))
            if len(pending) > 64:                                  # keep memory bounded
                n, d = pending.pop(0).result(); out[n] = d
        for f in pending:
            n, d = f.result(); out[n] = d
    return {"video": str(video), "size": (w // 2, h), "fps": fps, "exposure_s": expo, "frames": out}


# ---------------------------------------------------------------- choosing views

def blur_px(rec, n, eye):
    """How far this lens's corners move during one exposure around frame n, from the neighbouring decoded frames."""
    d = rec["frames"][n][eye]; speeds = []
    for m in (n - STEP, n + STEP):
        e = rec["frames"].get(m, [None, None])[eye]
        if e is None:
            continue
        common, ia, ib = np.intersect1d(d[0], e[0], return_indices=True)
        if len(common) >= 4:
            speeds.append(float(np.median(np.linalg.norm(d[1][ia] - e[1][ib], axis=1))) / (abs(m - n) / rec["fps"]))
    return None if not speeds else min(speeds) * rec["exposure_s"]


def distance(a, b):
    """Median distance between the corners two views share; inf when they share fewer than half of the smaller one's."""
    common, ia, ib = np.intersect1d(a[0], b[0], return_indices=True)
    if len(common) < 0.5 * min(len(a[0]), len(b[0])):
        return float("inf")
    return float(np.median(np.linalg.norm(a[1][ia] - b[1][ib], axis=1)))


def still(rec, eyes):
    """Frame numbers whose given lenses all see the board and are still enough."""
    keep = []
    for n in sorted(rec["frames"]):
        ok = True
        for eye in eyes:
            if rec["frames"][n][eye] is None:
                ok = False; break
            b = blur_px(rec, n, eye)
            if b is None or b > MAX_BLUR_PX:
                ok = False; break
        if ok:
            keep.append(n)
    return keep


def distinct(rec, candidates, eye, cap=MAX_VIEWS):
    """Greedy: take views with the most corners first, skip any within DISTINCT_PX of one already taken; then thin to cap
    by farthest-point order so the kept views stay spread out."""
    order = sorted(candidates, key=lambda n: -len(rec["frames"][n][eye][0]))
    kept = []
    for n in order:
        v = rec["frames"][n][eye]
        if all(distance(v, rec["frames"][k][eye]) > DISTINCT_PX for k in kept):
            kept.append(n)
    if len(kept) > cap:
        chosen = [kept[0]]; dmin = {k: distance(rec["frames"][k][eye], rec["frames"][kept[0]][eye]) for k in kept[1:]}
        while len(chosen) < cap and dmin:
            nxt = max(dmin, key=lambda k: min(dmin[k], 1e6))
            chosen.append(nxt); dmin.pop(nxt)
            for k in dmin:
                dmin[k] = min(dmin[k], distance(rec["frames"][k][eye], rec["frames"][nxt][eye]))
        kept = chosen
    return sorted(kept)


# ---------------------------------------------------------------- fits

def calibrate_lens(det, rec, views, eye):
    size = rec["size"]
    obj = [B.object_points(det, rec["frames"][n][eye][0]) for n in views]
    img = [rec["frames"][n][eye][1] for n in views]
    rms, K, D, rv, tv, _, _, per_view = cv2.calibrateCameraExtended(obj, img, size, None, None, flags=FLAGS, criteria=TERM)
    per_view = per_view.reshape(-1)
    bad = per_view > max(3 * float(np.median(per_view)), 1.0)
    if bad.any() and len(views) - bad.sum() >= 10:              # drop views that fit far worse than the rest, once
        views = [n for n, b in zip(views, bad) if not b]
        obj = [o for o, b in zip(obj, bad) if not b]; img = [i for i, b in zip(img, bad) if not b]
        rms, K, D, rv, tv, _, _, per_view = cv2.calibrateCameraExtended(obj, img, size, None, None, flags=FLAGS, criteria=TERM)
    tilt = [float(np.degrees(np.arccos(min(1.0, abs(cv2.Rodrigues(r)[0][2, 2]))))) for r in rv]
    dist_mm = [float(np.linalg.norm(t)) * 1000 for t in tv]
    cells = np.zeros((GRID, GRID), int)
    for i in img:
        cx = np.clip((i[:, 0] / size[0] * GRID).astype(int), 0, GRID - 1); cy = np.clip((i[:, 1] / size[1] * GRID).astype(int), 0, GRID - 1)
        np.add.at(cells, (cy, cx), 1)
    return {"K": K, "D": D.reshape(-1), "rms_px": float(rms), "views": views, "dropped_views": int(bad.sum()),
            "tilted_views": int(sum(t >= TILT_DEG for t in tilt)), "tilt_deg_max": round(max(tilt), 1),
            "distance_mm_min_median_max": [round(min(dist_mm)), round(float(np.median(dist_mm))), round(max(dist_mm))],
            "cells": int((cells >= CELL_MIN).sum()), "cell_counts": cells.tolist()}


def holdout_lens(det, rec, views, eye, K, D):
    """Reprojection error on views the model never saw: pose by solvePnP with the model fixed, then project."""
    errs = []
    for n in views:
        ids, pts = rec["frames"][n][eye]
        obj = B.object_points(det, ids)
        ok, rv, tv = cv2.solvePnP(obj, pts, K, D, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            continue
        rv, tv = cv2.solvePnPRefineLM(obj, pts, K, D, rv, tv)
        proj, _ = cv2.projectPoints(obj, rv, tv, K, D)
        errs.append(np.linalg.norm(proj.reshape(-1, 2) - pts, axis=1))
    e = np.concatenate(errs) if errs else np.array([np.nan])
    return {"views": len(errs), "corners": int(np.isfinite(e).sum()), "median_px": round(float(np.median(e)), 4),
            "p95_px": round(float(np.percentile(e, 95)), 4)}


def pairs(det, rec, views):
    obj, il, ir = [], [], []
    for n in views:
        (ia, pa), (ib, pb) = rec["frames"][n]
        common, xa, xb = np.intersect1d(ia, ib, return_indices=True)
        if len(common) >= MIN_CORNERS:
            obj.append(B.object_points(det, common)); il.append(pa[xa]); ir.append(pb[xb])
    return obj, il, ir


def stereo(det, rec, views, L, R):
    obj, il, ir = pairs(det, rec, views)
    rms, _, _, _, _, Rm, T, E, F = cv2.stereoCalibrate(obj, il, ir, L["K"], L["D"], R["K"], R["D"], rec["size"],
                                                       flags=FLAGS | cv2.CALIB_FIX_INTRINSIC, criteria=TERM)
    return {"R": Rm, "T": T.reshape(3), "E": E, "F": F, "rms_px": float(rms), "pairs": len(obj)}


def row_offset(det, rec, views, L, R, rect):
    """|row in left - row in right| after rectification, over every corner both lenses saw."""
    obj, il, ir = pairs(det, rec, views)
    dy = []
    for a, b in zip(il, ir):
        ua = cv2.undistortPoints(a.reshape(-1, 1, 2), L["K"], L["D"], R=rect["R1"], P=rect["P1"]).reshape(-1, 2)
        ub = cv2.undistortPoints(b.reshape(-1, 1, 2), R["K"], R["D"], R=rect["R2"], P=rect["P2"]).reshape(-1, 2)
        dy.append(np.abs(ua[:, 1] - ub[:, 1]))
    d = np.concatenate(dy) if dy else np.array([np.nan])
    return {"pairs": len(dy), "median_px": round(float(np.median(d)), 4), "p95_px": round(float(np.percentile(d, 95)), 4)}


# ---------------------------------------------------------------- checks

def check(name, value, limit, kind, plain):
    ok = value is not None and np.isfinite(value) and (value >= limit if kind == "min" else value <= limit)
    return {"name": f"{name} {'>=' if kind == 'min' else '<='} {limit}", "value": None if value is None else round(float(value), 4),
            "limit": limit, "pass": bool(ok), "plain": plain}


def lens_checks(lens, label):
    return [
        check(f"{label} views", len(lens["views"]), LIMITS["views"], "min",
              f"too few distinct still views of the board in the {label} lens (soft or dark picture, or the arm never stopped)"),
        check(f"{label} tilted views", lens["tilted_views"], LIMITS["tilted"], "min",
              f"too few views with the board turned {TILT_DEG:g} degrees or more in the {label} lens (the focal length is then poorly fixed)"),
        check(f"{label} cells", lens["cells"], LIMITS["cells"], "min",
              f"the board did not reach enough of the {label} lens's picture (distortion near the edges is then guessed)"),
        check(f"{label} fit rms px", lens["rms_px"], LIMITS["fit_rms_px"], "max",
              f"the lens model does not fit recording A in the {label} lens"),
    ]


def lens_part(det, rec, eye):
    views = distinct(rec, still(rec, (eye,)), eye)
    if len(views) < 6:
        return None, len(views)
    return calibrate_lens(det, rec, views, eye), len(views)


def first_check(rec_a, det):
    lenses, checks = {}, []
    for eye, label in ((0, "left"), (1, "right")):
        lens, n = lens_part(det, rec_a, eye)
        if lens is None:
            checks.append(check(f"{label} views", n, LIMITS["views"], "min", f"too few still views of the board in the {label} lens"))
            continue
        lenses[label] = lens
        checks += lens_checks(lens, label)
    return lenses, checks


# ---------------------------------------------------------------- output

def jsonable(lens):
    return {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in lens.items() if k != "views"} | {"views": len(lens["views"])}


def write_yaml(path, size, L, R, st, rect):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", int(size[0])); fs.write("image_height", int(size[1]))
    fs.write("distortion_model", MODEL)
    fs.write("board", json.dumps(B.BOARD))
    for k, v in (("camera_matrix_left", L["K"]), ("dist_coeffs_left", L["D"].reshape(1, -1)), ("camera_matrix_right", R["K"]),
                 ("dist_coeffs_right", R["D"].reshape(1, -1)), ("R", st["R"]), ("T", st["T"].reshape(3, 1)), ("E", st["E"]), ("F", st["F"]),
                 ("R1", rect["R1"]), ("R2", rect["R2"]), ("P1", rect["P1"]), ("P2", rect["P2"]), ("Q", rect["Q"])):
        fs.write(k, np.asarray(v, np.float64))
    fs.release()


def cache_key(video):
    s = Path(video).stat()
    return f"{Path(video).resolve()}|{s.st_size}|{int(s.st_mtime)}|{STEP}"


def save_detections(path, rec):
    data = {"key": cache_key(rec["video"]), "size": rec["size"], "fps": rec["fps"], "exposure_s": rec["exposure_s"],
            "frames": {str(n): [None if e is None else [e[0].tolist(), e[1].tolist()] for e in d] for n, d in rec["frames"].items()}}
    path.write_text(json.dumps(data))


def load_detections(path, video):
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError):
        return None
    if data.get("key") != cache_key(video):
        return None
    fr = {int(n): [None if e is None else (np.array(e[0], np.int32), np.array(e[1], np.float32)) for e in d] for n, d in data["frames"].items()}
    return {"video": str(video), "size": tuple(data["size"]), "fps": data["fps"], "exposure_s": data["exposure_s"], "frames": fr}


def recording(path, out, name):
    video = find_video(path)
    cache = out / f"detections-{name}.json"
    rec = load_detections(cache, video)
    if rec is None:
        log(f"{name}: decoding and finding the board in {video}")
        t = time.time(); rec = detect_recording(path)
        log(f"{name}: {len(rec['frames'])} frames in {time.time() - t:.0f} s, {rec['size'][0]} x {rec['size'][1]} per lens, "
            f"{rec['fps']:.1f} fps, exposure {rec['exposure_s'] * 1000:.1f} ms")
        save_detections(cache, rec)
    return rec


def finish(out, result):
    (out / "result.json").write_text(json.dumps(result, indent=1))
    print(json.dumps(result), flush=True)
    return 0 if result.get("pass") else 1


def cmd_check(a, out):
    t0 = time.time(); det = B.make_detector()
    rec = recording(a, out, "A"); t1 = time.time()
    log("A: fitting each lens")
    lenses, checks = first_check(rec, det)
    failed = [c for c in checks if not c["pass"]]
    res = {"stage": "check", "pass": not failed, "failed": [c["name"] for c in failed], "why": [c["plain"] for c in failed],
           "checks": checks, "size": rec["size"], "lenses_A": {k: jsonable(v) for k, v in lenses.items()},
           "frames_A": len(rec["frames"]), "board": B.BOARD, "model": MODEL,
           "seconds": {"detect_A": round(t1 - t0, 1), "fit_A": round(time.time() - t1, 1)}}
    return finish(out, res)


def cmd_solve(a, b, out):
    t0 = time.time(); det = B.make_detector(); secs = {}
    rec_a = recording(a, out, "A"); secs["detect_A"] = round(time.time() - t0, 1)
    t = time.time(); rec_b = recording(b, out, "B"); secs["detect_B"] = round(time.time() - t, 1)
    if rec_a["size"] != rec_b["size"]:
        return finish(out, {"stage": "solve", "pass": False, "failed": ["picture size"], "why": ["recordings A and B have different picture sizes"]})
    t = time.time(); log("A: fitting each lens")
    lenses, checks = first_check(rec_a, det); secs["fit_lenses_A"] = round(time.time() - t, 1)
    res = {"stage": "solve", "size": rec_a["size"], "board": B.BOARD, "model": MODEL, "frames": {"A": len(rec_a["frames"]), "B": len(rec_b["frames"])},
           "lenses_A": {k: jsonable(v) for k, v in lenses.items()}}
    if len(lenses) == 2:
        L, R = lenses["left"], lenses["right"]
        t = time.time(); log("B: held-out error per lens")
        hold = {}
        for eye, label, lens in ((0, "left", L), (1, "right", R)):
            hold[label] = holdout_lens(det, rec_b, distinct(rec_b, still(rec_b, (eye,)), eye, cap=10 ** 6), eye, lens["K"], lens["D"])
            checks.append(check(f"{label} held-out median px", hold[label]["median_px"], LIMITS["holdout_median_px"], "max",
                                f"lens model error too high in the {label} lens on recording B, which it never saw"))
        secs["holdout_B"] = round(time.time() - t, 1)
        t = time.time(); log("A: stereo fit")
        pa = distinct(rec_a, still(rec_a, (0, 1)), 0)
        checks.append(check("A pairs", len(pairs(det, rec_a, pa)[0]), LIMITS["pairs"], "min", "too few still views with the board in both lenses in recording A"))
        pb = distinct(rec_b, still(rec_b, (0, 1)), 0, cap=10 ** 6)
        checks.append(check("B pairs", len(pairs(det, rec_b, pb)[0]), LIMITS["pairs"], "min", "too few still views with the board in both lenses in recording B"))
        if all(c["pass"] for c in checks if c["name"].startswith(("A pairs", "B pairs"))):
            st = stereo(det, rec_a, pa, L, R)
            R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(L["K"], L["D"], R["K"], R["D"], rec_a["size"], st["R"], st["T"],
                                                              flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
            rect = {"R1": R1, "R2": R2, "P1": P1, "P2": P2, "Q": Q}
            secs["stereo_A"] = round(time.time() - t, 1); t = time.time(); log("B: held-out row offset and repeat")
            dy_b = row_offset(det, rec_b, pb, L, R, rect); dy_a = row_offset(det, rec_a, pa, L, R, rect)
            st_b = stereo(det, rec_b, distinct(rec_b, still(rec_b, (0, 1)), 0), L, R)
            rot = float(np.degrees(np.linalg.norm(cv2.Rodrigues(st["R"] @ st_b["R"].T)[0])))
            repeat_mm = float(np.linalg.norm(st["T"] - st_b["T"])) * 1000
            fx, fy = P1[0, 0], P1[1, 1]; w, h = rec_a["size"]
            res["stereo"] = {"baseline_mm": round(float(np.linalg.norm(st["T"])) * 1000, 3), "T_mm": [round(float(v) * 1000, 3) for v in st["T"]],
                             "rotation_deg": round(float(np.degrees(np.linalg.norm(cv2.Rodrigues(st["R"])[0]))), 4),
                             "fit_rms_px_A": round(st["rms_px"], 4), "pairs_A": st["pairs"],
                             "rect_dy_A": dy_a, "rect_dy_B": dy_b,
                             "repeat": {"baseline_mm_B": round(float(np.linalg.norm(st_b["T"])) * 1000, 3), "translation_change_mm": round(repeat_mm, 3),
                                        "rotation_change_deg": round(rot, 4), "pairs_B": st_b["pairs"]},
                             "rectified_fov_deg": [round(float(np.degrees(2 * np.arctan(w / (2 * fx)))), 1), round(float(np.degrees(2 * np.arctan(h / (2 * fy)))), 1)]}
            checks.append(check("rectified row offset median px", dy_b["median_px"], LIMITS["rect_dy_median_px"], "max",
                                "left and right rows do not line up after rectification on recording B"))
            checks.append(check("repeat mm", repeat_mm, LIMITS["repeat_mm"], "max",
                                "recordings A and B give a different right-lens position (did the camera or the board move?)"))
            write_yaml(out / "calibration.yaml", rec_a["size"], L, R, st, rect)
            secs["check_B"] = round(time.time() - t, 1)
        res["holdout_B"] = hold
    failed = [c for c in checks if not c["pass"]]
    secs["total"] = round(time.time() - t0, 1)
    res.update({"pass": not failed and "stereo" in res, "failed": [c["name"] for c in failed], "why": [c["plain"] for c in failed],
                "checks": checks, "seconds": secs, "calibration_file": str(out / "calibration.yaml") if "stereo" in res else None})
    return finish(out, res)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=("check", "solve"))
    ap.add_argument("recordings", nargs="+")
    ap.add_argument("--out", required=True, type=Path)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    if a.what == "check" and len(a.recordings) == 1:
        sys.exit(cmd_check(a.recordings[0], a.out))
    if a.what == "solve" and len(a.recordings) == 2:
        sys.exit(cmd_solve(a.recordings[0], a.recordings[1], a.out))
    ap.error("check takes one recording, solve takes two")


if __name__ == "__main__":
    main()
