#!/usr/bin/env python3
"""Quality check, separate from calibration: horizontal banding (faint stripes along the sensor rows).

  qc_banding.py <recording video or picture> [...]     prints one JSON line per file

Per lens, per frame: mean brightness of every row inside each of 8 vertical strips; subtract a 13-row running mean (keeps
ripple with a period of about 2 to 12 rows); take, for every row, the MEDIAN across the 8 strips, so scene detail that
differs from strip to strip drops out and a stripe that crosses the whole sensor stays; the score is the standard deviation
of that common ripple in grey levels. A recording's score is the median over frames spread through it.

The station also scores still live pictures before recording (calib_station.still_banding): a moving scene and video
compression hide banding in a recording, so a still picture shows it more clearly. Both limits are provisional: cameras
without visible banding have scored about 0.2 to 0.4 in recordings; set the limits from your own cameras' numbers.
"""
import json, subprocess, sys
import numpy as np

LIMIT = 1.0           # grey levels, recording score
STRIPS, WINDOW = 8, 13


def frame_score(eye):
    g = eye.astype(np.float32); sw = g.shape[1] // STRIPS
    rows = np.stack([g[:, i * sw:(i + 1) * sw].mean(1) for i in range(STRIPS)], 1)
    k = np.ones(WINDOW, np.float32) / WINDOW
    hp = rows - np.stack([np.convolve(np.pad(rows[:, i], WINDOW // 2, mode="edge"), k, mode="valid") for i in range(STRIPS)], 1)
    return float(np.median(hp, axis=1).std())


def size_of(path):
    out = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=width,height", "-of", "csv=p=0",
                          str(path)], capture_output=True, text=True).stdout.strip().split(",")
    return int(out[0]), int(out[1])


def check_video(path, every=40):
    """{"pass", "left", "right", "limit", "frames"} for one side-by-side recording (median score per lens)."""
    try: width, height = size_of(path)
    except (ValueError, IndexError): return {"pass": None, "error": "the video could not be read"}
    p = subprocess.Popen(["ffmpeg", "-v", "error", "-nostdin", "-i", str(path), "-vf", f"select=not(mod(n\\,{every}))", "-fps_mode", "passthrough",
                          "-f", "image2pipe", "-pix_fmt", "gray", "-vcodec", "rawvideo", "-"], stdout=subprocess.PIPE)
    l, r = [], []
    while True:
        raw = p.stdout.read(width * height)
        if len(raw) < width * height: break
        g = np.frombuffer(raw, np.uint8).reshape(height, width); l.append(frame_score(g[:, :width // 2])); r.append(frame_score(g[:, width // 2:]))
    p.stdout.close(); p.wait()
    if not l: return {"pass": None, "error": "no frames could be read"}
    left, right = float(np.median(l)), float(np.median(r))
    return {"pass": bool(max(left, right) <= LIMIT), "left": round(left, 3), "right": round(right, 3), "limit": LIMIT, "frames": len(l),
            "worst_frame": round(float(max(max(l), max(r))), 3)}


if __name__ == "__main__":
    for path in sys.argv[1:]:
        print(json.dumps({"file": path.split("/")[-1], **check_video(path)}))
