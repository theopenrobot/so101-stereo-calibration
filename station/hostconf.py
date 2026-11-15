"""Where things are on this machine. Every station program imports this first, so the same files run on a Mac and on a
Linux box (a Raspberry Pi 5) with nothing edited.

Settings are read from the environment, and before that from a settings file of KEY=VALUE lines (copy
station.env.example to station.env at the top of the repo; STATION_ENV_FILE names another file). A value already in
the environment wins.

  STATION_PY           Python that runs the arm programs, the picture check and the solver   default: this Python
  STATION_CAMERA_URL   the camera program (camera/stereo_stream.py)                     default: http://127.0.0.1:8090
  STATION_DATA         runtime data: runs/, station/ (log, results, settings)          default: <repo>/data
  SO101_IN             folder holding the arm programs                                 default: <repo>/arm
  STATION_NAME         this station's name in the session log and on the page          default: this machine's name
  STATION_PORT / DASH_PORT / DASH_BIND   the station program's and the page's ports, and where the page listens
                                         default: 18772 / 18771 / 127.0.0.1

Nothing here talks to hardware.
"""
import os
import shutil
import socket
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
ENV_FILE = Path(os.environ.get("STATION_ENV_FILE") or REPO / "station.env")


def _load_env_file(path=ENV_FILE):
    try:
        lines = path.read_text().splitlines()
    except OSError:
        return
    for line in lines:
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        os.environ.setdefault(key.strip(), os.path.expanduser(value.strip().strip('"').strip("'")))


_load_env_file()

PY = os.environ.get("STATION_PY") or sys.executable
CAMERA_URL = (os.environ.get("STATION_CAMERA_URL") or "http://127.0.0.1:8090").rstrip("/")
DATA = Path(os.environ.get("STATION_DATA") or REPO / "data").expanduser()
ARM_DIR = Path(os.environ.get("SO101_IN") or REPO / "arm")
ARM_CWD = REPO                                     # the arm programs are started from the top of the repo
STATION_NAME = os.environ.get("STATION_NAME") or socket.gethostname().split(".")[0]
STATION_PORT = int(os.environ.get("STATION_PORT", "18772"))
DASH_PORT = int(os.environ.get("DASH_PORT", "18771"))
DASH_BIND = os.environ.get("DASH_BIND", "127.0.0.1")
STATION_URL = f"http://127.0.0.1:{STATION_PORT}"


def _speech_command():
    if sys.platform == "darwin" and os.path.exists("/usr/bin/say"):
        return ["/usr/bin/say"]
    for name in ("espeak-ng", "espeak", "spd-say"):
        path = shutil.which(name)
        if path:
            return [path]
    return None


def say(words):
    """Speak a cue without waiting. Silent on a machine with no speech program, or with STATION_SPEECH=0."""
    cmd = _speech_command()
    if cmd is None or os.environ.get("STATION_SPEECH") == "0":
        return
    try:
        subprocess.Popen([*cmd, words], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError:
        pass


def report():
    return {"platform": sys.platform, "env_file": str(ENV_FILE) if ENV_FILE.exists() else None, "python": PY,
            "camera_url": CAMERA_URL, "data": str(DATA), "arm_dir": str(ARM_DIR), "station_name": STATION_NAME,
            "station_port": STATION_PORT, "dash_port": DASH_PORT, "dash_bind": DASH_BIND,
            "speech": _speech_command(), "ffmpeg": shutil.which("ffmpeg")}


if __name__ == "__main__":                         # python station/hostconf.py : show what this machine resolves to
    import json
    print(json.dumps(report(), indent=1))
