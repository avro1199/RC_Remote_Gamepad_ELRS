import time
from pymavlink import mavutil


# ===== EDIT THIS =====
PORT = "COM16"            # Windows example: "COM7"
# PORT = "/dev/ttyACM0"  # Linux example
BAUD = 115200            # common for USB; try 57600 if heartbeat not seen
# =====================


def connect_usb(port: str, baud: int) -> mavutil.mavfile:
    master = mavutil.mavlink_connection(port, baud=baud)

    print(f"Connecting on {port} @ {baud} ... waiting for HEARTBEAT")
    master.wait_heartbeat(timeout=20)
    print(f"Heartbeat OK: sys={master.target_system} comp={master.target_component}")
    return master


def wait_command_ack(master: mavutil.mavfile, command_id: int, timeout_s: float = 3.0):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout_s)
        if msg and msg.command == command_id:
            return msg
    return None


def command_long(master: mavutil.mavfile, cmd: int, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, timeout_s=3.0):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        cmd,
        0,  # confirmation
        float(p1), float(p2), float(p3), float(p4), float(p5), float(p6), float(p7)
    )
    return wait_command_ack(master, cmd, timeout_s)


def set_mode(master: mavutil.mavfile, mode_name: str):
    mapping = master.mode_mapping()
    if not mapping or mode_name not in mapping:
        raise RuntimeError(f"Mode '{mode_name}' not available. Mapping: {mapping}")

    mode_id = mapping[mode_name]
    master.set_mode(mode_id)

    # quick verify by reading a heartbeat (optional)
    hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    if hb:
        print(f"Mode set requested: {mode_name} (custom_mode={hb.custom_mode})")


def arm(master: mavutil.mavfile, do_arm: bool):
    # MAV_CMD_COMPONENT_ARM_DISARM = 400
    ack = command_long(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1 if do_arm else 0)
    print("ARM/DISARM ACK:", ack)
    return ack


def takeoff(master: mavutil.mavfile, alt_m: float = 10.0):
    # For ArduCopter: go GUIDED then arm, then takeoff
    # MAV_CMD_NAV_TAKEOFF = 22 (p7 = altitude)
    ack = command_long(master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=alt_m)
    print("TAKEOFF ACK:", ack)
    return ack


def beep(master, frequency_hz=2500, duration_s=2.0):
    """
    MAV_CMD_DO_SET_TONE (183)
      p1: frequency (Hz)
      p2: duration (ms)
      p3: silence (ms) - optional
    """
    MAV_CMD_DO_SET_TONE = 183
    ack = command_long(
        master,
        MAV_CMD_DO_SET_TONE,
        p1=float(frequency_hz),
        p2=float(int(duration_s * 1000)),
        p3=0
    )
    print("TONE ACK:", ack)
    return ack


def main():
    master = connect_usb(PORT, BAUD)

    # Example workflow
    # 1) set GUIDED
    set_mode(master, "STABILIZE")  # or "GUIDED" if supported; STABILIZE is more universal for testing

    # 2) arm
    arm(master, True)

    # 3) send an example 'custom' command (beep/tone)
    beep(master, 2500, 2.0)

    # 4) (optional) takeoff
    # takeoff(master, 10)

    # 5) disarm (comment out if you're flying)
    # arm(master, False)


if __name__ == "__main__":
    main()