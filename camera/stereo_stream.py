#!/usr/bin/env python3
"""Camera program: live view and recordings of a USB stereo camera, over HTTP. Linux (a Raspberry Pi 5 or a PC).

Plain V4L2 and ffmpeg, standard library otherwise; nothing to install beyond ffmpeg and v4l-utils. The camera must
deliver MJPEG with both lenses side by side in one frame (left half = left lens). ffmpeg copies the camera's JPEG
frames out untouched (no re-encoding, almost no CPU); a recording is those same frames written one after the other to
video.mjpeg, with one line per frame in frames.csv.

    python3 stereo_stream.py [--device /dev/video0] [--size 3840x1200] [--fps 30] [--port 8090]
    open http://<this machine>:8090/        live page

GET  /status          JSON: frame rate, frame size, frame age, recording state, owner
GET  /camera          JSON: is a camera plugged in, its id, sizes offered, exposure, USB speed
GET  /frame.jpg       newest full frame          GET /preview  small fast MJPEG stream    GET /stream  full MJPEG stream
GET  /buttons         Start / Stop push buttons on a Pi's pin header (optional)
GET  /record/tar?name=<recording>                one finished recording as a plain tar
POST /exposure?us=<250..10000|auto>              fixed exposure through the V4L2 controls
POST /record/start?name=<name>&seconds=<s>       POST /record/stop     POST /record/delete?name=<name>
POST /led?state=<waiting|busy|ready|moving|pass|fail|problem|off>      a Pi's two board lights as the station's status
POST /lease           the owning station says it is still there

Every POST must name its station in an X-Station header. The first station to send one owns this program until it has
been silent for OWNER_IDLE_S seconds; commands from any other station are refused, so two stations never change the
exposure under each other's recording. The exposure cannot change while a recording runs.

The camera id (used by the station to tell cameras apart) is CAMERA_ID when set, else the USB serial number the
camera reports. Many cameras of one model report the same USB serial: then set CAMERA_ID per camera, or give each a
unique serial.
"""
import argparse, csv, json, os, re, shutil, subprocess, tarfile, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

p = argparse.ArgumentParser()
p.add_argument("--device", default=os.environ.get("CAMERA_DEVICE", "/dev/video0"))
p.add_argument("--size", default=os.environ.get("CAMERA_SIZE", ""), help="WxH; default: the widest MJPEG size the camera offers")
p.add_argument("--fps", type=int, default=int(os.environ.get("CAMERA_FPS", "30")))
p.add_argument("--port", type=int, default=int(os.environ.get("CAMERA_PORT", "8090")))
p.add_argument("--bind", default=os.environ.get("CAMERA_BIND", "0.0.0.0"))
args = p.parse_args()

REC_ROOT = os.path.expanduser(os.environ.get("CAMERA_REC_DIR", "~/stereo-frames/rec"))
OWNER = {"id": None, "t": 0.0}
OWNER_IDLE_S = 120


class Latest:
    def __init__(self):
        self.cond = threading.Condition(); self.frame = None; self.seq = 0
        self.t0 = time.time(); self.count = 0; self.bytes = 0; self.rate = 0.0; self.avg = 0; self.error = ""
        self.viewers = 0; self.t = 0.0; self.size = args.size or "?"
    def put(self, jpg):
        with self.cond:
            self.frame = jpg; self.seq += 1; self.count += 1; self.bytes += len(jpg); self.t = time.time()
            now = time.time()
            if now - self.t0 >= 2.0:
                self.rate = self.count / (now - self.t0); self.avg = self.bytes // max(1, self.count)
                self.t0, self.count, self.bytes = now, 0, 0
            self.cond.notify_all()
    def wait(self, after, timeout=5.0):
        with self.cond:
            if self.seq <= after: self.cond.wait(timeout)
            return self.frame, self.seq

latest = Latest()
preview = Latest()         # small, fast pictures for watching live: about 1280 wide, 15 a second


def jpeg_end(buf, hint=0):
    """Length of the complete JPEG at the start of buf, 0 if more bytes are needed, -1 if buf does not start a JPEG.

    The header is walked segment by segment using each segment's length, because table data can contain the
    bytes FF D9 by chance. Only after the scan header is the end marker searched for.
    """
    n = len(buf)
    if n < 4: return 0
    if buf[0] != 0xFF or buf[1] != 0xD8: return -1
    i = 2
    while True:
        if i + 4 > n: return 0
        if buf[i] != 0xFF: return -1
        m = buf[i + 1]
        if m == 0xFF: i += 1; continue                         # fill byte
        seg = (buf[i + 2] << 8) | buf[i + 3]
        if seg < 2: return -1
        i += 2 + seg
        if m == 0xDA: break                                    # start of scan: compressed data follows
    # Inside compressed data FF is always followed by 00 or a restart marker, so the first FF D9 after the scan
    # header is the true end. `hint` lets the caller skip bytes already searched.
    j = buf.find(b"\xff\xd9", max(i, hint))
    return 0 if j < 0 else j + 2


def read_jpegs(stream, sink, also=None):
    """Split a pipe of back-to-back JPEGs into frames."""
    buf = bytearray()
    try:
        while True:
            chunk = stream.read(65536)
            if not chunk: break
            searched = max(0, len(buf) - 1); buf += chunk
            while True:
                end = jpeg_end(buf, searched)
                if end == 0: break
                if end < 0: del buf[:1]; searched = 0; continue    # not at a frame start: resynchronise
                jpg = bytes(buf[:end]); sink.put(jpg)
                if also: also(jpg)
                del buf[:end]; searched = 0
    except (OSError, ValueError): pass


# ---------------------------------------------------------------- V4L2 controls

CTRL = {"names": None}


def v4l2(*a, timeout=5):
    try: return subprocess.run(["v4l2-ctl", "-d", args.device, *a], capture_output=True, text=True, timeout=timeout)
    except Exception as e: return subprocess.CompletedProcess(a, 1, "", str(e))


def controls():
    """Names of the exposure controls on this kernel: newer ones say auto_exposure / exposure_time_absolute, older ones
    exposure_auto / exposure_absolute. Both count exposure_time_absolute in units of 100 microseconds."""
    if CTRL["names"] is None:
        out = v4l2("-l").stdout
        auto = "auto_exposure" if "auto_exposure" in out else "exposure_auto"
        absol = "exposure_time_absolute" if "exposure_time_absolute" in out else "exposure_absolute"
        if absol not in out: return None
        CTRL["names"] = (auto, absol)
    return CTRL["names"]


EXPO = {"us": None}       # the fixed exposure last asked for; set again when the stream re-opens the camera


def set_exposure(us):
    """us = None: automatic. Returns (ok, reply)."""
    c = controls()
    if c is None: return False, "this camera offers no V4L2 exposure control"
    auto, absol = c
    r = v4l2("-c", f"{auto}=3") if us is None else v4l2("-c", f"{auto}=1", "-c", f"{absol}={max(1, round(us / 100))}")
    return r.returncode == 0, (r.stdout + r.stderr).strip()[:200]


def read_exposure_us():
    c = controls()
    if c is None: return None
    m = re.search(r":\s*(\d+)", v4l2("-C", c[1]).stdout)
    return int(m.group(1)) * 100 if m else None


def mjpeg_sizes():
    f = v4l2("--list-formats-ext").stdout
    mj = f.split("'MJPG'")[1].split("[")[0] if "'MJPG'" in f else ""
    return sorted(set(re.findall(r"Size: Discrete (\d+x\d+)", mj)), key=lambda s: [int(v) for v in s.split("x")])


def best_size():
    """The size asked for, else the widest MJPEG size offered (the full side-by-side sensor). Asked up to three times:
    right after a camera is plugged in the format list can come back empty."""
    if args.size: return args.size
    for _ in range(3):
        s = mjpeg_sizes()
        if s: return max(s, key=lambda x: (int(x.split("x")[0]), int(x.split("x")[1])))
        time.sleep(1.0)
    return "1280x480"


# ---------------------------------------------------------------- recording

REC = {"name": None, "dir": None, "fh": None, "csv": None, "csvfh": None, "frames": 0, "until": 0.0, "started": 0.0,
       "exposure_us": None, "result": None, "error": ""}
rec_lock = threading.Lock()


def rec_write(jpg):
    """Called for every camera frame. While a recording runs, appends the frame untouched to video.mjpeg."""
    with rec_lock:
        if REC["name"] is None: return
        try:
            if REC["frames"] % 300 == 0 and shutil.disk_usage(REC["dir"]).free < 2_000_000_000: raise OSError("less than 2 GB free")
            off = REC["fh"].tell(); REC["fh"].write(jpg)
            REC["csv"].writerow([REC["frames"], time.time_ns(), off, len(jpg), REC["exposure_us"] or ""])
            REC["frames"] += 1
        except OSError as e:
            REC["error"] = str(e)[:200]; rec_close_locked()
            return
        if time.time() >= REC["until"]: rec_close_locked()


def rec_close_locked():
    if REC["name"] is None: return
    try: REC["fh"].close(); REC["csvfh"].close()
    except OSError: pass
    secs = max(0.001, time.time() - REC["started"]); rate = REC["frames"] / secs
    checks = []
    if REC["error"]: checks.append("recording stopped early: " + REC["error"])
    if REC["frames"] == 0: checks.append("no frames were recorded")
    elif rate < 0.8 * args.fps: checks.append(f"only {rate:.1f} frames a second were recorded, {args.fps} expected")
    verdict = "PASS" if not checks else "FAIL"
    summary = {"verdict": verdict, "checks": checks, "frames": REC["frames"], "seconds": round(secs, 2), "fps": round(rate, 2)}
    info = {"device": args.device, "size": latest.size, "camera": camera_info(fast=True), "exposure_us": REC["exposure_us"], "summary": summary}
    try:
        with open(os.path.join(REC["dir"], "device.json"), "w") as f: json.dump(info, f, indent=1)
    except OSError: pass
    REC["result"] = {"ok": verdict == "PASS", "frames": REC["frames"], "verdict": verdict + (": " + "; ".join(checks) if checks else "")}
    REC["name"] = None


def rec_start(name, seconds):
    with rec_lock:
        if REC["name"] is not None: return {"ok": False, "error": "already recording " + REC["name"]}
        if time.time() - latest.t > 2.0: return {"ok": False, "error": "no picture from the camera"}
        d = os.path.join(REC_ROOT, name); os.makedirs(d, exist_ok=False)
        fh = open(os.path.join(d, "video.mjpeg"), "wb"); csvfh = open(os.path.join(d, "frames.csv"), "w", newline="")
        w = csv.writer(csvfh); w.writerow(["frame", "host_ns", "offset", "size", "exposure_us"])
        REC.update(name=name, dir=d, fh=fh, csv=w, csvfh=csvfh, frames=0, started=time.time(), until=time.time() + seconds,
                   exposure_us=read_exposure_us() if EXPO["us"] else None, result=None, error="")
        return {"ok": True, "dir": d, "seconds": seconds}


def rec_stop():
    with rec_lock:
        if REC["name"] is None: return {"ok": False, "error": "not recording"}
        name = REC["name"]; rec_close_locked()
        return {"ok": True, "name": name, "result": REC["result"]}


# ---------------------------------------------------------------- the camera stream

def capture():
    while True:
        if not os.path.exists(args.device):              # no camera plugged in: wait for one
            latest.error = "no camera"; CTRL["names"] = None; time.sleep(0.5); continue
        latest.size = best_size()
        pr, pw = os.pipe()                               # second output: the same pictures made small, for the fast live view
        cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "v4l2", "-input_format", "mjpeg", "-video_size", latest.size,
               "-framerate", str(args.fps), "-i", args.device, "-map", "0:v", "-c", "copy", "-f", "mjpeg", "pipe:1",
               "-map", "0:v", "-vf", "fps=15,scale=1280:-2", "-q:v", "6", "-f", "mjpeg", f"pipe:{pw}"]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0, pass_fds=(pw,), preexec_fn=lambda: os.nice(10))
        os.close(pw); pstream = os.fdopen(pr, "rb", buffering=0)
        threading.Thread(target=read_jpegs, args=(pstream, preview), daemon=True).start()
        if EXPO["us"]:                                   # opening the camera can put exposure back to automatic
            threading.Timer(1.0, lambda: set_exposure(EXPO["us"])).start()
        try:
            read_jpegs(proc.stdout, latest, rec_write)
        finally:
            proc.kill()
            try: proc.wait(timeout=3)
            except Exception: pass
            err = proc.stderr.read().decode("utf-8", "replace").strip().splitlines()
            latest.error = err[-1][:200] if err else "camera stream ended"
            try: pstream.close()
            except Exception: pass
            with rec_lock:
                if REC["name"] is not None: REC["error"] = "the camera stream ended"; rec_close_locked()
        time.sleep(1.0)


CAMC = {"key": None, "serial": None, "sizes": [], "usb_speed": None}


def camera_info(fast=False):
    """Which camera is plugged in right now: its id, the sizes it offers, its exposure, and how fresh the last frame is."""
    out = {"present": os.path.exists(args.device), "frame_age_s": round(time.time() - latest.t, 2) if latest.t else None, "size": latest.size}
    if not out["present"]: return out
    try: key = os.stat(args.device).st_ctime
    except OSError: key = None
    if CAMC["key"] != key:                               # read once per plug-in
        serial = speed = None
        try:
            dev = os.path.realpath(f"/sys/class/video4linux/{os.path.basename(args.device)}/device/..")
            serial = open(os.path.join(dev, "serial")).read().strip() if os.path.exists(os.path.join(dev, "serial")) else None
            speed = open(os.path.join(dev, "speed")).read().strip()
        except OSError: pass
        CAMC.update(key=key, serial=serial, usb_speed=speed, sizes=[] if fast else mjpeg_sizes())
    out.update(serial=CAMC["serial"], usb_speed=CAMC["usb_speed"], sizes=CAMC["sizes"],
               identity=os.environ.get("CAMERA_ID") or CAMC["serial"])
    if not fast: out["exposure_us"] = read_exposure_us()
    return out


# ---------------------------------------------------------------- a Pi's lights and buttons (optional)

# The Pi's two lights as the station's status. Green is the ACT light, red is the PWR light.
#   waiting  green, slow wink      fit a camera               busy     green, quick blink    working: hands off, wait
#   ready    green, steady         press Start                moving   red, blinking         the arm is moving
#   pass     green, double flash                              fail     red, steady
#   problem  red and green steady  the station needs attention
LED = {"state": "off", "ok": False, "error": "not started"}
LED_STEPS = {"off": ([(0, 1.0)], [(0, 1.0)]), "waiting": ([(1, 0.12), (0, 1.9)], [(0, 1.0)]), "busy": ([(1, 0.15), (0, 0.15)], [(0, 1.0)]),
             "ready": ([(1, 1.0)], [(0, 1.0)]), "moving": ([(0, 1.0)], [(1, 0.25), (0, 0.25)]),
             "pass": ([(1, 0.12), (0, 0.12), (1, 0.12), (0, 0.9)], [(0, 1.0)]), "fail": ([(0, 1.0)], [(1, 1.0)]), "problem": ([(1, 1.0)], [(1, 1.0)])}


def led_loop():
    """Drives both lights by writing their brightness files. Needs write access to them; without it, it only reports why."""
    files = {"g": "/sys/class/leds/ACT/brightness", "r": "/sys/class/leds/PWR/brightness"}; last = {"g": None, "r": None}
    t0 = time.time(); shown = None; wrote = {"g": 0.0, "r": 0.0}
    while True:
        try:
            state = LED["state"]
            if state != shown: shown = state; t0 = time.time()
            for key, steps in zip(("g", "r"), LED_STEPS.get(state, LED_STEPS["off"])):
                period = sum(d for _, d in steps); x = (time.time() - t0) % period; v = steps[-1][0]
                for val, d in steps:
                    if x < d: v = val; break
                    x -= d
                if v != last[key] or time.time() - wrote[key] > 1.0:      # also re-assert every second, in case something else reset it
                    with open(files[key], "w") as f: f.write(str(v))
                    last[key] = v; wrote[key] = time.time()
            LED.update(ok=True, error="")
        except Exception as e:
            LED.update(ok=False, error=f"{type(e).__name__}: {str(e)[:100]}"); last = {"g": None, "r": None}; time.sleep(2)
        time.sleep(0.03)


# Start and Stop push buttons on the Pi's pin header. Each button joins its pin to a ground pin when pressed; the Pi's own
# pull-up holds the pin high otherwise. START: pin 11 (GPIO 17) to pin 9 (ground). STOP: pin 13 (GPIO 27) to pin 14 (ground).
BTN = {"ok": False, "error": "not started", "boot": f"{os.getpid()}-{int(time.time())}", "start_low": False, "stop_low": False,
       "start_falls": 0, "stop_falls": 0, "stop_rises": 0, "t": 0.0,
       "power_taps": 0, "power_ok": False, "power_error": "not started", "power_down": False}
START_PIN, STOP_PIN = 17, 27


def power_button():
    """The Pi 5's own button. A tap (pressed for under 1.5 s) counts once; the station treats it as Start when ready and
    Stop while the arm moves. Stop any other service that acts on this button first. Holding it about 10 s still cuts
    power in hardware, whatever software is running."""
    import struct
    fmt = "llHHi"; size = struct.calcsize(fmt); down_at = None
    while True:
        try:
            with open("/dev/input/by-path/platform-pwr_button-event", "rb", buffering=0) as f:
                BTN.update(power_ok=True, power_error="")
                while True:
                    ev = f.read(size)
                    if len(ev) < size: break
                    _, _, etype, code, value = struct.unpack(fmt, ev)
                    if etype != 1 or code != 116: continue          # EV_KEY, KEY_POWER
                    if value == 1: down_at = time.time(); BTN["power_down"] = True
                    elif value == 0:
                        BTN["power_down"] = False
                        if down_at is not None and time.time() - down_at < 1.5: BTN["power_taps"] += 1
                        down_at = None
        except Exception as e:
            BTN.update(power_ok=False, power_error=f"{type(e).__name__}: {str(e)[:120]}")
        time.sleep(2)


def buttons():
    """Watch the two pins 100 times a second. A change counts only after it has held for 30 ms, so contact bounce is ignored."""
    try:
        import gpiod
        from gpiod.line import Direction, Bias, Value
        cfg = gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_UP)
        req = gpiod.request_lines("/dev/gpiochip0", consumer="station-buttons", config={START_PIN: cfg, STOP_PIN: cfg})
    except Exception as e:
        BTN.update(ok=False, error=f"{type(e).__name__}: {str(e)[:120]}"); return
    BTN.update(ok=True, error="")
    stable = {START_PIN: None, STOP_PIN: None}; pending = {START_PIN: (None, 0.0), STOP_PIN: (None, 0.0)}
    while True:
        try:
            now = time.time(); vals = req.get_values([START_PIN, STOP_PIN])
            for pin, v in zip((START_PIN, STOP_PIN), vals):
                low = v == Value.INACTIVE
                if stable[pin] is None: stable[pin] = low; continue
                if low == stable[pin]: pending[pin] = (None, 0.0); continue
                if pending[pin][0] != low: pending[pin] = (low, now); continue
                if now - pending[pin][1] >= 0.03:
                    stable[pin] = low; pending[pin] = (None, 0.0)
                    if pin == START_PIN and low: BTN["start_falls"] += 1
                    if pin == STOP_PIN: BTN["stop_falls" if low else "stop_rises"] += 1
            BTN.update(start_low=bool(stable[START_PIN]), stop_low=bool(stable[STOP_PIN]), t=now, ok=True, error="")
        except Exception as e:
            BTN.update(ok=False, error=f"{type(e).__name__}: {str(e)[:120]}")
        time.sleep(0.01)


# ---------------------------------------------------------------- HTTP

PAGE = b"""<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1"><title>Stereo live</title>
<style>body{margin:0;background:#0b0d10;color:#e8eaed;font:15px system-ui,sans-serif}header{display:flex;gap:12px;align-items:center;flex-wrap:wrap;padding:10px 14px;background:#15181d}
#info{margin-left:auto;color:#9aa0a6;font-variant-numeric:tabular-nums}img{display:block;width:100%}</style>
<header><b>Stereo camera, live</b><span id=info>connecting...</span></header><img id=v src="/preview">
<script>const info=document.getElementById('info');
setInterval(async()=>{try{const s=await (await fetch('/status')).json();info.textContent=s.size+' - '+s.camera_fps.toFixed(1)+' fps - '+Math.round(s.frame_bytes/1024)+' KB per frame'+(s.recording?' - RECORDING '+s.recording:'')+(s.error?' - '+s.error:'')}catch(e){info.textContent='no answer'}},1500);
</script>"""
NAME = re.compile(r"[A-Za-z0-9_.-]{1,80}")


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    def log_message(self, *a): pass
    def _send(self, code, ctype, body):
        if not isinstance(body, bytes): body = json.dumps(body).encode()
        self.send_response(code); self.send_header("Content-Type", ctype); self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store"); self.send_header("Access-Control-Allow-Origin", "*"); self.end_headers(); self.wfile.write(body)
    def _mjpeg(self, src, gap=0.0):
        self.send_response(200); self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store"); self.send_header("Access-Control-Allow-Origin", "*"); self.send_header("Connection", "close"); self.end_headers()
        seq = 0; sent = 0.0; src.viewers += 1
        try:
            while True:
                frame, seq2 = src.wait(seq, 2.0)
                if frame is None or seq2 == seq: continue
                seq = seq2
                if gap and time.time() - sent < gap * 0.9: continue
                sent = time.time()
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
        except (BrokenPipeError, ConnectionResetError, OSError): pass
        finally: src.viewers -= 1
        self.close_connection = True
    def do_GET(self):
        u = urlparse(self.path); q = parse_qs(u.query); J = "application/json"
        if u.path == "/": return self._send(200, "text/html; charset=utf-8", PAGE)
        if u.path == "/status":
            return self._send(200, J, {"size": latest.size, "frame_age_s": round(time.time() - latest.t, 2) if latest.t else None,
                                       "camera_fps": round(latest.rate, 1), "frame_bytes": latest.avg, "frames": latest.seq, "viewers": latest.viewers,
                                       "preview_fps": round(preview.rate, 1), "recording": REC["name"],
                                       "record": {"name": REC["name"], "running": REC["name"] is not None, "frames": REC["frames"], "result": REC["result"]},
                                       "error": "" if latest.rate > 1 else latest.error,
                                       "owner": OWNER["id"] if time.time() - OWNER["t"] < OWNER_IDLE_S else None, "exposure_fixed_us": EXPO["us"]})
        if u.path == "/led": return self._send(200, J, LED)
        if u.path == "/buttons": return self._send(200, J, {**BTN, "age_s": round(time.time() - BTN["t"], 3) if BTN["t"] else None})
        if u.path == "/camera": return self._send(200, J, camera_info())
        if u.path == "/frame.jpg":
            frame, _ = latest.wait(-1, 3.0)
            if frame and time.time() - latest.t > 2.0: frame = None      # the camera has gone: never serve a stale picture
            return self._send(200, "image/jpeg", frame) if frame else self._send(503, "text/plain", b"no frame from the camera")
        if u.path == "/preview.jpg":
            frame = preview.frame if time.time() - preview.t < 3.0 else None
            return self._send(200, "image/jpeg", frame) if frame else self._send(503, "text/plain", b"no live picture")
        if u.path == "/preview": return self._mjpeg(preview)
        if u.path == "/stream": return self._mjpeg(latest, 1.0 / max(1, min(60, int(q.get("fps", ["15"])[0]))))
        if u.path == "/record/tar":                    # download one finished recording as a plain tar
            name = q.get("name", [""])[0]; d = os.path.join(REC_ROOT, name)
            if not NAME.fullmatch(name) or not os.path.isdir(d) or REC["name"] == name:
                return self._send(404, "text/plain", b"no such finished recording")
            self.send_response(200); self.send_header("Content-Type", "application/x-tar"); self.send_header("Connection", "close"); self.end_headers()
            try:
                with tarfile.open(fileobj=self.wfile, mode="w|") as tar: tar.add(d, arcname=name)
            except (BrokenPipeError, ConnectionResetError, OSError): pass
            self.close_connection = True
            return
        self._send(404, "text/plain", b"not found")
    def do_POST(self):
        u = urlparse(self.path); q = parse_qs(u.query); J = "application/json"
        who = (self.headers.get("X-Station") or "").strip()[:80]
        if not who: return self._send(403, J, {"ok": False, "error": "commands must name their station (X-Station header)"})
        if OWNER["id"] not in (None, who) and time.time() - OWNER["t"] < OWNER_IDLE_S:
            return self._send(409, J, {"ok": False, "error": f"the camera program belongs to station {OWNER['id']}"})
        OWNER.update(id=who, t=time.time())
        if u.path == "/lease": return self._send(200, J, {"ok": True, "owner": who})
        if u.path == "/exposure":
            if REC["name"] is not None: return self._send(409, J, {"ok": False, "error": "a recording is running: the exposure cannot change now"})
            us = q.get("us", [""])[0]
            if us == "auto": ok, reply = set_exposure(None); EXPO["us"] = None if ok else EXPO["us"]
            elif us.isdigit() and 250 <= int(us) <= 10000:
                ok, reply = set_exposure(int(us))
                if ok: EXPO["us"] = int(us)
            else: ok, reply = False, "us must be 250..10000 or auto"
            return self._send(200, J, {"ok": ok, "reply": reply, "readback_us": read_exposure_us()})
        if u.path == "/led":
            st = q.get("state", [""])[0]
            if st in LED_STEPS: LED["state"] = st
            return self._send(200, J, LED)
        if u.path.startswith("/record/"):
            name = q.get("name", [""])[0]
            try:
                if u.path == "/record/start":
                    if not NAME.fullmatch(name): raise ValueError("bad name")
                    body = rec_start(name, max(2, min(300, int(q.get("seconds", ["40"])[0]))))
                elif u.path == "/record/stop": body = rec_stop()
                elif u.path == "/record/delete":
                    if not NAME.fullmatch(name) or REC["name"] == name: raise ValueError("bad name")
                    shutil.rmtree(os.path.join(REC_ROOT, name)); body = {"ok": True}
                else: return self._send(404, "text/plain", b"not found")
            except Exception as e:
                body = {"ok": False, "error": f"{type(e).__name__}: {str(e)[:160]}"}
            return self._send(200, J, body)
        self._send(404, "text/plain", b"not found")


if __name__ == "__main__":
    os.makedirs(REC_ROOT, exist_ok=True)
    for target in (capture, buttons, power_button, led_loop):
        threading.Thread(target=target, daemon=True).start()
    print(f"camera program on {args.bind}:{args.port}, device {args.device}, recordings in {REC_ROOT}", flush=True)
    ThreadingHTTPServer((args.bind, args.port), Handler).serve_forever()
