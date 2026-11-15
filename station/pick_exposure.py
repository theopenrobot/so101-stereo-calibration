"""Choose a fixed exposure for the camera now on the arm.

Tries exposures from 0.4 to 10 ms, keeps those where the board is seen in both lenses, and picks the longest one that does
not wash out the board's white squares. Prints one JSON line. Exit 1 if none is usable.

  python pick_exposure.py              full sweep of exposures
  python pick_exposure.py --only 4000  set this one exposure (microseconds) and check the board with it
"""
import json, os, sys, time, urllib.error, urllib.request
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import hostconf as H                                          # noqa: E402
import board as B                                             # noqa: E402
import cv2, numpy as np                                       # noqa: E402

ONLY = int(sys.argv[sys.argv.index("--only") + 1]) if "--only" in sys.argv else None
CAM = H.CAMERA_URL; det = B.make_detector(); tried = []
MIN_SEEN = int(os.environ.get("PICK_MIN_CORNERS", str(int(0.8 * B.CORNERS))))     # corners each lens must see


def post(p):
    try:
        req = urllib.request.Request(CAM + p, method="POST", headers={"X-Station": os.environ.get("STATION_ID", "unnamed")})
        return json.loads(urllib.request.urlopen(req, timeout=12).read())
    except urllib.error.HTTPError as e:            # 403/409: this station may not command the camera program now; say so plainly and stop
        try: why = json.loads(e.read()).get("error")
        except Exception: why = f"HTTP {e.code}"
        print(json.dumps({"chosen_us": None, "chosen": None, "tried": [], "error": f"the camera program refused: {why}"})); sys.exit(1)


def grab(): return cv2.imdecode(np.frombuffer(urllib.request.urlopen(CAM + "/frame.jpg", timeout=6).read(), np.uint8), 0)


def measure(us):
    """Set the exposure, wait until the picture has really changed and settled, then score one frame. None if the camera refused."""
    if not post(f"/exposure?us={us}").get("ok"): return None
    time.sleep(1.0); g = grab(); t0 = time.time()
    while time.time() - t0 < 4.0:                       # settled = two frames a quarter second apart with the same brightness
        time.sleep(0.25); h = grab()
        if abs(float(h.mean()) - float(g.mean())) <= 0.03 * max(float(g.mean()), 1.0): g = h; break
        g = h
    W = g.shape[1] // 2; eyes = []
    for k in (0, 1):
        img = g[:, k * W:(k + 1) * W]; d = B.detect(det, img); e = {"corners": 0 if d is None else int(len(d[0])), "sharpness": round(B.sharpness(img))}
        if d is not None and len(d[0]) >= 12:           # light levels on the board itself, not on the window or the room behind it
            pts = d[1]; x0, y0 = pts.min(0).astype(int); x1, y1 = pts.max(0).astype(int); b = img[y0:y1, x0:x1]
            if b.size: e.update(board_white=int(np.percentile(b, 95)), board_black=int(np.percentile(b, 5)), board_clipped_pct=round(100 * float((b >= 250).mean()), 1))
        eyes.append(e)
    return {"us": us, "eyes": eyes, "size": [W, int(g.shape[0])], "brightness": round(float(g.mean()), 1)}


try: before = json.loads(urllib.request.urlopen(CAM + "/status", timeout=6).read()).get("exposure_fixed_us")
except Exception: before = None
for us in ((ONLY,) if ONLY else (400, 700, 1000, 1500, 2500, 4000, 6000, 10000)):
    t = measure(us)
    if t: tried.append(t)
# A longer exposure must give a brighter picture. Where it does not, the camera had not taken the setting when the frame was
# read (then a 10 ms picture can be scored as "0.4 ms"). Measure those again; drop them if still wrong.
for i in range(len(tried) - 2, -1, -1):
    if tried[i]["brightness"] > tried[i + 1]["brightness"] * 1.02:
        again = measure(tried[i]["us"])
        if again and again["brightness"] <= tried[i + 1]["brightness"] * 1.02: tried[i] = again
        else: tried[i]["eyes"] = [{"corners": 0, "sharpness": 0}, {"corners": 0, "sharpness": 0}]; tried[i]["rejected"] = "brighter than a longer exposure: the setting had not taken"
ok = [t for t in tried if min(e["corners"] for e in t["eyes"]) >= MIN_SEEN]
# Which exposure: the longest one that does not wash out the board's white squares. Not "sharpest standing still": many
# cameras raise their own gain at short exposures, the noise that adds inflates the sharpness score, and that rule then keeps
# choosing short, noisy exposures. At this arm speed, with the arm holding still at each view, a long exposure has cost
# nothing in kept frames.
def washed(t): return max(e.get("board_clipped_pct", 100.0) for e in t["eyes"])
for t in tried: t["board_clipped_pct"] = washed(t)
clean = [t for t in ok if washed(t) <= 1.0]
best = max(clean, key=lambda t: t["us"]) if clean else (min(ok, key=washed) if ok else None)
if best: post(f"/exposure?us={best['us']}")
elif before: post(f"/exposure?us={before}")          # nothing usable: put the camera back as it was, not at the last exposure tried
print(json.dumps({"chosen_us": best["us"] if best else None, "chosen": best, "min_corners": MIN_SEEN, "of": B.CORNERS,
                  "rule": "longest exposure with no more than 1 percent of the board washed out", "tried": tried}))
sys.exit(0 if best else 1)
