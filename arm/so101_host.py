"""What differs between a Mac and a Linux box (a Raspberry Pi 5) for the arm programs, in one place.

The arm programs ask this module instead of naming devices themselves:

  follower_port() / leader_port()   the serial device of each arm's control board
  profile_path()                    the white (follower) arm's five-joint calibration profile
  port_users(port)                  who else has the serial device open (the programs refuse to start if anyone does)
  say(words)                        speak a cue without waiting, when this machine has a speech program

How a port is found, first match wins:

  1. SO101_FOLLOWER_PORT / SO101_LEADER_PORT, or follower_port / leader_port in arm/arm.json: a device path.
  2. SO101_FOLLOWER_SERIAL / SO101_LEADER_SERIAL, or follower_serial / leader_serial in arm/arm.json: the USB serial
     number of that arm's control board (python arm/so101_host.py lists the boards it sees, with their serials).
  3. The USB vendor:product id of the control board (SO101_USB_ID or usb_id in arm/arm.json, default 1a86:55d3, the
     CH343 USB-serial chip on the common SO-101 bus servo board). With exactly one such board plugged in, it is taken
     as the follower. The leader is never guessed this way: with two boards, give at least the follower's serial.

Settings come from the environment first, then arm/arm.json (copy arm.example.json), then the defaults above.
SO101_PROFILE (or "profile" in arm.json) is the calibration profile, default arm/profile/calibration.json.

Nothing here writes to a motor or opens a serial device: listing USB devices does not open them.
"""
import glob
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
_CONF_FILE = ROOT / "arm.json"
DEFAULT_USB_ID = "1a86:55d3"


def _conf():
    try:
        return json.loads(_CONF_FILE.read_text())
    except (OSError, ValueError):
        return {}


def _setting(env, key, default=None):
    return os.environ.get(env) or _conf().get(key) or default


def boards():
    """Every USB serial device on this machine: [{"device", "vid_pid", "serial", "description"}]. Opens nothing."""
    out = []
    try:
        from serial.tools import list_ports                 # pyserial, installed with lerobot
        for p in list_ports.comports():
            if p.vid is None:
                continue
            dev = p.device
            if sys.platform == "darwin" and dev.startswith("/dev/tty."):
                dev = "/dev/cu." + dev[len("/dev/tty."):]   # on a Mac the call-out device is the one to open
            out.append({"device": dev, "vid_pid": f"{p.vid:04x}:{p.pid:04x}", "serial": p.serial_number or "",
                        "description": p.description or ""})
    except ImportError:                                     # without pyserial: device names only, no USB ids
        pattern = "/dev/cu.usbmodem*" if sys.platform == "darwin" else "/dev/serial/by-id/*"
        out = [{"device": d, "vid_pid": "", "serial": "", "description": ""} for d in sorted(glob.glob(pattern))]
    return out


def _port(which):
    direct = _setting(f"SO101_{which.upper()}_PORT", f"{which}_port")
    if direct:
        return direct
    found = boards()
    serial = _setting(f"SO101_{which.upper()}_SERIAL", f"{which}_serial")
    if serial:
        mine = [b["device"] for b in found if b["serial"] and serial in b["serial"]]
        if not mine:                                         # pyserial missing or no serial reported: try the device name
            mine = sorted(set(glob.glob(f"/dev/cu.usbmodem{serial}*") + glob.glob(f"/dev/serial/by-id/*{serial}*")))
        if len(mine) != 1:
            raise RuntimeError(f"The {which} arm (USB serial {serial}) is {'not plugged in' if not mine else 'ambiguous: ' + ', '.join(mine)}.")
        return mine[0]
    usb_id = _setting("SO101_USB_ID", "usb_id", DEFAULT_USB_ID).lower()
    same = [b for b in found if b["vid_pid"] == usb_id]
    if which == "follower" and len(same) == 1:
        return same[0]["device"]
    if which == "leader":
        other = _setting("SO101_FOLLOWER_SERIAL", "follower_serial")
        rest = [b for b in same if not (other and other in b["serial"])]
        if other and len(rest) == 1:
            return rest[0]["device"]
    seen = ", ".join(f"{b['device']} ({b['vid_pid']}, serial {b['serial'] or '?'})" for b in found) or "none"
    raise RuntimeError(f"The {which} arm's port could not be chosen: {len(same)} board(s) with USB id {usb_id}. "
                       f"Set {which}_serial or {which}_port in {_CONF_FILE} (or SO101_{which.upper()}_SERIAL / "
                       f"SO101_{which.upper()}_PORT). USB serial devices seen: {seen}.")


def follower_port():
    return _port("follower")


def leader_port():
    return _port("leader")


def profile_path():
    p = Path(_setting("SO101_PROFILE", "profile", "profile/calibration.json")).expanduser()
    return p if p.is_absolute() else ROOT / p


def port_users(port):
    """Who has this serial device open, as lsof -t would report it: returncode 0 with pids on stdout when someone
    does, returncode 1 with empty stdout when nobody does, any other returncode when it could not be checked.
    The callers treat "could not be checked" as busy, so a missing tool never lets two programs share an arm."""
    lsof = shutil.which("lsof") or next((p for p in ("/usr/sbin/lsof", "/usr/bin/lsof") if os.path.exists(p)), None)
    if lsof:
        return subprocess.run([lsof, "-t", port], capture_output=True, text=True)
    if not os.path.isdir("/proc/self/fd"):
        return subprocess.CompletedProcess(["port_users", port], 2, "", "no lsof and no /proc on this machine")
    try:
        target = os.path.realpath(port)
        if not os.path.exists(target):
            return subprocess.CompletedProcess(["port_users", port], 2, "", f"{port} does not exist")
        pids = []
        for fd_dir in glob.glob("/proc/[0-9]*/fd"):
            try:
                names = os.listdir(fd_dir)
            except OSError:                                    # another user's process, or one that just ended
                continue
            for name in names:
                try:
                    if os.path.realpath(os.path.join(fd_dir, name)) == target:
                        pids.append(fd_dir.split("/")[2])
                        break
                except OSError:
                    pass
        return subprocess.CompletedProcess(["port_users", port], 0 if pids else 1, "".join(p + "\n" for p in pids), "")
    except OSError as e:
        return subprocess.CompletedProcess(["port_users", port], 2, "", str(e))


def _speech_command():
    if sys.platform == "darwin" and os.path.exists("/usr/bin/say"):
        return ["/usr/bin/say"]
    for name in ("espeak-ng", "espeak", "spd-say"):
        path = shutil.which(name)
        if path:
            return [path]
    return None


def say(words):
    """Speak without waiting (the control loop must keep its period). Silent on a machine with no speech program.
    Raises OSError when the speech program cannot be started, as the direct call did."""
    cmd = _speech_command()
    if cmd is None:
        return None
    return subprocess.Popen([*cmd, words], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == "__main__":                                     # python so101_host.py : show what this machine resolves to
    out = {"platform": sys.platform, "profile": str(profile_path()), "profile_exists": profile_path().exists(),
           "speech": _speech_command(), "usb_serial_devices": boards()}
    for which in ("follower", "leader"):
        try:
            out[which] = _port(which)
        except RuntimeError as e:
            out[which] = f"unavailable: {e}"
    print(json.dumps(out, indent=1))
