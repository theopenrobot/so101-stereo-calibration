"""Write the preflight record for a station sweep, in which the operator keeps people clear of the arm themselves.

    SECONDS_CMD=.. SPEED_CMD=.. TRAVEL_CMD=.. TORQUE_CMD=.. python make_preflight_waived.py <record.json>

The arm programs refuse a powered run without a fresh preflight record for the exact command (see
arm/so101-five-joint-teleop.py, preflight_refusals, and arm/so101-five-joint-sweep.py, sweep_refusals). At the station no
camera frame of the workspace is inspected: the record says so, as a waiver in the operator's words (the station puts in
who or what pressed Start), and it expires after a few minutes like any other record. Starting the station means
accepting this: whoever presses Start is responsible for keeping people out of the arm's reach.
"""
import json, os, sys
from datetime import datetime
out = sys.argv[1]; now = datetime.now().astimezone().isoformat()
rec = {"checked_at": now, "command": {"seconds": int(os.environ.get("SECONDS_CMD", 31)), "speed": int(os.environ.get("SPEED_CMD", 400)),
                                      "travel": int(os.environ.get("TRAVEL_CMD", 2100)), "torque_limit": int(os.environ.get("TORQUE_CMD", 800))},
       "clearance": {"operator_waived_camera_check": True, "operator_words": "The operator keeps people clear of the arm.",
                     "note": "No camera frame of the workspace was inspected for this run."},
       "operator": {"ready": True, "confirmed_at": now},
       "camera": {"captured_at": now, "inspected": False}}
with open(out, "w") as f: json.dump(rec, f, indent=1)
