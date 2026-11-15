#!/usr/bin/env python3
"""Calibration station console: the operator page (calib_console.html), the live camera view with board corners, and the
Sessions screen. Sessions come only from sessions.py (ARCHITECTURE.md).

Read-only. It pulls frames from the camera program and reads the station's session log. It never talks to the arm; Start,
Stop and Set home are passed to the station program.

    <python> calib_dashboard.py        then open http://127.0.0.1:18771/ (DASH_PORT, DASH_BIND)
"""
import json, os, sys, threading, time, urllib.error, urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import cv2, numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import hostconf as H                                                      # noqa: E402
import board as B                                                         # noqa: E402
import sessions as SES                                                    # noqa: E402

CAM = H.CAMERA_URL
LIVE_EVERY = float(os.environ.get("LIVE_DOTS_EVERY", "0.25"))             # seconds between corner checks of the live picture (slower on a Pi)
live = {"ok": False, "error": "starting", "eyes": {}, "keep": False, "cam": {}, "jpg": None, "t": 0, "dots": {}, "aspect": [3840, 1200], "cam_url": CAM}
lock = threading.Lock()
BUSY = ("recording_a", "recording_b", "first_check", "solving", "qc", "checking")   # no live corner finding while these run


def cam_status():
    try: return json.loads(urllib.request.urlopen(CAM + "/status", timeout=3).read())
    except Exception: return {}


def live_loop():
    det = B.make_detector()
    while True:
        try:
            try: busy = json.loads(urllib.request.urlopen(H.STATION_URL + "/state", timeout=2).read()).get("phase") in BUSY
            except Exception: busy = False
            if busy:                                        # the arm or the solver is working: leave them the processor
                st = cam_status()
                with lock: live.update(ok=bool(st), error="", cam=st, dots={}, t=time.time(), keep=False, busy=True)
                time.sleep(1.0); continue
            with lock: live["busy"] = False
            raw = urllib.request.urlopen(CAM + "/frame.jpg", timeout=4).read(); st = cam_status()
            im = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR); g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
            Hh, W2 = g.shape; W = W2 // 2; eyes, keep, dots = {}, True, {}
            for k, eye in enumerate(("cam0", "cam1")):
                img = g[:, k * W:(k + 1) * W]; r = B.detect(det, img); sh = B.sharpness(img)
                eyes[eye] = {"corners": 0 if r is None else int(len(r[0])), "sharpness": round(sh)}
                keep &= (r is not None) and sh >= B.SHARP_MIN
                dots[eye] = [] if r is None else [[round(float(x) / W, 4), round(float(y) / Hh, 4)] for x, y in r[1]]
            with lock: live.update(ok=True, error="", eyes=eyes, keep=bool(keep), cam=st, t=time.time(), dots=dots, aspect=[W2, Hh])
        except urllib.error.HTTPError as e:                 # no frame (camera absent or busy): keep the camera program's report fresh
            st = cam_status()
            with lock: live.update(ok=bool(st), error="" if st else f"HTTPError: {e.code}", cam=st, dots={}, t=time.time(), keep=False)
            time.sleep(0.5)
        except Exception as e:
            with lock: live.update(ok=False, error=f"{type(e).__name__}: {str(e)[:100]}", t=time.time())
            time.sleep(1.0)
        time.sleep(LIVE_EVERY)


PAGE = HERE / "calib_console.html"
FONTS = HERE / "fonts"


def sessions():
    """Every session, newest first (sessions.py), with the numbers the Sessions screen compares."""
    out = []
    for e in SES.load(H.DATA / "station" / "log.jsonl"):
        row = SES.summary_row(e); qc = e.get("qc_banding") or {}
        if e.get("lens_error_px") and all(v is not None for v in e["lens_error_px"]):
            row["m"] = {"lens_l": e["lens_error_px"][0], "lens_r": e["lens_error_px"][1], "rows": e.get("rect_dy_px"),
                        "cells": min(e["cells"]) if e.get("cells") and None not in e["cells"] else None, "repeat_mm": e.get("repeat_mm"),
                        "baseline": e.get("baseline_mm"), "banding": max(qc["left"], qc["right"]) if qc.get("left") is not None else None}
        out.append(row)
    return out[::-1]


class Hd(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_POST(self):
        p = self.path.split("?")[0]
        if p in ("/api/station/start", "/api/station/stop", "/api/station/sethome"):
            try: body = urllib.request.urlopen(urllib.request.Request(H.STATION_URL + "/" + p.rsplit("/", 1)[1], method="POST"), timeout=40).read()
            except Exception as e: body = json.dumps({"ok": False, "why": f"The station program is not answering ({type(e).__name__})."}).encode()
            return self.send(200, "application/json", body)
        self.send(404, "text/plain", b"not found")
    def send(self, code, ctype, body):
        self.send_response(code); self.send_header("Content-Type", ctype); self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(body)
    def do_GET(self):
        p = self.path.split("?")[0]; parts = [x for x in p.split("/") if x]
        try:
            if p == "/": return self.send(200, "text/html; charset=utf-8", PAGE.read_bytes())
            if p == "/api/page":                          # the console reloads itself when this changes, so nobody runs an old page
                return self.send(200, "application/json", json.dumps({"version": int(PAGE.stat().st_mtime)}).encode())
            if p == "/api/sessions": return self.send(200, "application/json", json.dumps(sessions()).encode())
            if len(parts) == 2 and parts[0] == "fonts" and (FONTS / parts[1]).is_file() and parts[1].endswith((".woff2", ".css")):
                return self.send(200, "text/css" if parts[1].endswith(".css") else "font/woff2", (FONTS / parts[1]).read_bytes())
            if p == "/api/live":
                with lock: d = {k: v for k, v in live.items() if k != "jpg"}
                if "127.0.0.1" in CAM or "localhost" in CAM:      # camera program on this machine: the browser reaches it by the address it used for this page
                    d["cam_url"] = "http://" + (self.headers.get("Host") or "127.0.0.1").split(":")[0] + ":" + CAM.rsplit(":", 1)[1]
                return self.send(200, "application/json", json.dumps(d).encode())
            if p == "/api/station":
                try: body = urllib.request.urlopen(H.STATION_URL + "/state", timeout=3).read()
                except Exception: body = json.dumps({"phase": "offline", "message": "The station program is not running."}).encode()
                return self.send(200, "application/json", body)
        except Exception as e:
            return self.send(500, "text/plain", f"{type(e).__name__}: {e}".encode()[:300])
        self.send(404, "text/plain", b"not found")


if __name__ == "__main__":
    threading.Thread(target=live_loop, daemon=True).start()
    print(f"console on http://{H.DASH_BIND}:{H.DASH_PORT}/  (station {H.STATION_URL}, camera program {CAM})", flush=True)
    ThreadingHTTPServer((H.DASH_BIND, H.DASH_PORT), Hd).serve_forever()
