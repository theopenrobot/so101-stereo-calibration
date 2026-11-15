"""The one definition of a calibration session and of the numbers counted from them.

Every screen and count reads sessions through this module: the station's header and session list (calib_station.py) and the
Sessions screen (calib_dashboard.py). Nothing else parses station/log.jsonl. Change a definition here, and every screen changes.

A session is one press of Start (or one automatic start) for one camera. Its three stages, each True, False or None (not reached):
  check   recording A alone passed the solver's first check (enough distinct and tilted views, the board reached the picture)
  solve   the pair was solved and every calibration check passed (held-out error, row offset, repeat)
  qc      the banding quality check on recording A passed
A camera "passed" when it has at least one session with outcome "passed" (all three True).
"""
import json
from datetime import datetime
from pathlib import Path


def stages_of(e):
    fc = e.get("first_check"); sol = e.get("solve"); qc = e.get("qc_banding")
    return {"check": None if not fc else bool(fc.get("pass")),
            "solve": None if not sol else bool(sol.get("pass")),
            "qc": None if not qc or qc.get("pass") is None else bool(qc["pass"])}


def load(log):
    """All sessions in the log, oldest first. One bad line never hides the rest."""
    out = []
    try: lines = Path(log).read_text().splitlines()
    except OSError: return out
    for ln in lines:
        try: e = json.loads(ln)
        except ValueError: continue
        if not isinstance(e, dict) or not e.get("time") or not e.get("camera"): continue
        e.setdefault("session", f"{e['camera']}-{e['time'][:19].replace('-', '').replace(':', '').replace('T', '-')}")
        e["stages"] = stages_of(e)
        e["passed"] = e.get("outcome") == "passed"
        out.append(e)
    return out


def today(sessions, now=None):
    """Counts for the header, for the local calendar day of `now`: cameras passed, cameras tried, sessions run."""
    day = (now or datetime.now().astimezone()).strftime("%Y-%m-%d")
    mine = [s for s in sessions if s["time"].startswith(day)]
    return {"day": day, "cameras_passed": len({s["camera"] for s in mine if s["passed"]}), "cameras_tried": len({s["camera"] for s in mine}),
            "sessions": len(mine), "sessions_failed": sum(1 for s in mine if s.get("outcome") == "failed"),
            "sessions_error": sum(1 for s in mine if s.get("outcome") in ("error", "stopped"))}


def summary_row(e):
    """The fields a session list shows."""
    return {k: e.get(k) for k in ("session", "time", "camera", "outcome", "seconds", "checks_passed", "why", "reason", "lens_error_px",
                                  "timing", "stages", "passed", "exposure_us", "exposure_recorded_us", "result_dir")} | {"recorded": bool(e.get("runs"))}
