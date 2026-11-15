"""Read-only register dump of the black SO-101 leader arm, joints 1-5.

Never writes registers, enables torque or sends targets. Motor 6 is not addressed.
Each read keeps the raw error byte of the status packet, so a servo fault flag is
recorded instead of raised.
"""

from datetime import datetime
import json
from pathlib import Path
import runpy
import subprocess

from lerobot.motors.motors_bus import get_address


root = Path(__file__).resolve().parent
import sys as _sys; _sys.path.insert(0, str(Path(__file__).resolve().parent)); import so101_host as HOST   # Mac / Linux differences: ports, busy-port check
safe = runpy.run_path(str(root / "so101-calibrate-safe.py"))
PORT = HOST.leader_port()
REGISTERS = (
    "Model_Number", "Firmware_Major_Version", "Firmware_Minor_Version", "ID", "Baud_Rate", "Max_Voltage_Limit", "Min_Voltage_Limit",
    "Torque_Enable", "Status", "Moving", "Present_Position", "Goal_Position",
    "Min_Position_Limit", "Max_Position_Limit", "Homing_Offset", "Torque_Limit",
    "Max_Torque_Limit", "Lock", "Operating_Mode", "Present_Load", "Present_Current",
    "Present_Voltage", "Present_Temperature", "Unloading_Condition", "LED_Alarm_Condition",
    "Response_Status_Level", "Acceleration", "Goal_Velocity",
)


class ReadOnlyBus(safe["FeetechMotorsBus"]):
    def write(self, *args, **kwargs):
        raise RuntimeError("Read-only inspection: register writes blocked")

    def _write(self, *args, **kwargs):
        raise RuntimeError("Read-only inspection: register writes blocked")

    def sync_write(self, *args, **kwargs):
        raise RuntimeError("Read-only inspection: motor targets blocked")

    def _sync_write(self, *args, **kwargs):
        raise RuntimeError("Read-only inspection: motor targets blocked")


result = {"checked_at": datetime.now().astimezone().isoformat(), "read_only": True, "port": PORT}
used = HOST.port_users(PORT)
if used.returncode not in (0, 1) or used.stdout.strip():
    raise SystemExit(f"Port ownership check failed or busy: {used.stdout} {used.stderr}")

device = safe["SO101Follower"](safe["SO101FollowerConfig"](port=PORT, id="follower"))
motors = {name: motor for name, motor in device.bus.motors.items() if name != "gripper"}
bus = ReadOnlyBus(PORT, motors)
try:
    bus.connect(handshake=False)
    result["registers"] = {}
    result["ids_found"] = bus.broadcast_ping(raise_on_error=False) if hasattr(bus, "broadcast_ping") else None
    for name, motor in motors.items():
        row = result["registers"][name] = {}
        for register in REGISTERS:
            address, length = get_address(bus.model_ctrl_table, motor.model, register)
            value, comm, error = bus._read(address, length, motor.id, num_retry=2, raise_on_error=False)
            if not bus._is_comm_success(comm):
                row[register] = {"comm": bus.packet_handler.getTxRxResult(comm)}
                continue
            row[register] = bus._decode_sign(register, {motor.id: value})[motor.id]
            if error:
                row.setdefault("packet_error_bytes", {})[register] = error
except Exception as error:
    result["error"] = f"{type(error).__name__}: {error}"
finally:
    if bus.is_connected:
        bus.disconnect(disable_torque=False)

path = root / f"so101-black-registers-{datetime.now():%Y%m%d-%H%M%S-%f}.json"
path.write_text(json.dumps(result, indent=2))
names = list(result.get("registers", {}))
print(f"{'register':<24}" + "".join(f"{name:>15}" for name in names))
for register in REGISTERS:
    print(f"{register:<24}" + "".join(f"{str(result['registers'][n].get(register)):>15}" for n in names))
print("packet_error_bytes", {n: result["registers"][n].get("packet_error_bytes") for n in names})
print("error", result.get("error"))
print(f"Evidence: {path}")
print("Port released. No register writes, torque enable or movement commands.")
