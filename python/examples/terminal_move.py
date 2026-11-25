import time
import numpy as np

from client import ElegooRobotCar


# ---------------------------------------------------------------------------
# Adjustable Defaults
# ---------------------------------------------------------------------------
ROBOT_IP = "192.168.4.1"
DEFAULT_DURATION = 0.5   # Default move duration (sec) if omitted in command
V_DRIVE = 30.0           # Linear speed for forward/backward (cm/s)
PWM_TURN = 140           # Wheel PWM magnitude for in-place left/right turns
LR_RATIO = 1.0           # Left/right wheel ratio for straight-line driving

PAN_CENTER_DEG = 90
PAN_LEFT_DEG = 0         # Flipped: 0 deg = left, 180 deg = right
PAN_RIGHT_DEG = 180
PAN_STEP_DEG = 30


def v2pwm(v: float) -> int:
    """Map linear velocity (cm/s) to integer motor PWM in [-255, 255]."""
    if abs(v) < 1e-6:
        return 0
    pwm_val = int(np.trunc(3.0207 * v - 1.73)) if v > 0 else int(np.trunc(3.0207 * v + 1.73))
    return int(np.clip(pwm_val, -255, 255))


def _find_numeric(data: object, keys: tuple[str, ...]) -> float | None:
    """Recursively search a dict/list for the first matching numeric key."""
    if isinstance(data, dict):
        lower_map = {str(k).lower(): v for k, v in data.items()}
        for key in keys:
            k_low = key.lower()
            if k_low in lower_map and isinstance(lower_map[k_low], (int, float)) and not isinstance(lower_map[k_low], bool):
                return float(lower_map[k_low])
        for v in data.values():
            found = _find_numeric(v, keys)
            if found is not None:
                return found
    elif isinstance(data, list):
        for item in data:
            found = _find_numeric(item, keys)
            if found is not None:
                return found
    return None


def extract_distance_cm(reading: object) -> float | None:
    if isinstance(reading, (int, float)) and not isinstance(reading, bool):
        return float(reading)
    return _find_numeric(reading, ("distance", "dist", "cm", "range", "value"))


def extract_imu_fields(reading: object) -> tuple[float | None, float | None]:
    """Return (omega_z, acceleration_y) from read_imu() payload."""
    omega_z = _find_numeric(reading, ("omega_z", "wz", "gz", "gyro_z", "gyroz", "z_gyro", "yaw_rate"))
    accel_y = _find_numeric(reading, ("acceleration_y", "accel_y", "ay", "acc_y", "accy", "y_accel", "y"))
    return omega_z, accel_y


def extract_line_sensors(reading: object) -> object:
    """Extract line-tracking sensor values from read_ir() payload."""
    if isinstance(reading, dict):
        for key in ("ir", "line", "sensors", "values", "data", "track"):
            if key in reading and reading[key] is not None:
                return reading[key]
    return reading


def print_sensors(robot: ElegooRobotCar) -> None:
    dist_raw = robot.read_distance()
    imu_raw = robot.read_imu()
    ir_raw = robot.read_ir()

    dist_cm = extract_distance_cm(dist_raw)
    omega_z, accel_y = extract_imu_fields(imu_raw)
    line_vals = extract_line_sensors(ir_raw)

    dist_str = f"{dist_cm:.2f} cm" if dist_cm is not None else f"raw={dist_raw}"
    wz_str = f"{omega_z:+.4f}" if omega_z is not None else f"raw={imu_raw}"
    ay_str = f"{accel_y:+.4f}" if accel_y is not None else f"raw={imu_raw}"

    print(f"  distance       : {dist_str}")
    print(f"  omega_z        : {wz_str}")
    print(f"  acceleration_y : {ay_str}")
    print(f"  line sensors   : {line_vals}")

    if dist_cm is None:
        print(f"  [debug read_distance]: {dist_raw}")
    if omega_z is None or accel_y is None:
        print(f"  [debug read_imu]     : {imu_raw}")


def run_timed_drive(robot: ElegooRobotCar, left_pwm: int, right_pwm: int, duration: float) -> None:
    """Drive with (left_pwm, right_pwm) for `duration` seconds, then stop."""
    left_pwm = int(np.clip(int(left_pwm), -255, 255))
    right_pwm = int(np.clip(int(right_pwm), -255, 255))
    duration = max(0.0, float(duration))

    if duration == 0.0:
        robot.stop()
        return

    t_start = time.perf_counter()
    try:
        while (time.perf_counter() - t_start) < duration:
            robot.drive(left_pwm, right_pwm)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nMove interrupted — stopping.")
    finally:
        try:
            robot.stop()
        except Exception:
            pass


def main() -> None:
    fwd_pwm = v2pwm(abs(V_DRIVE))
    left_fwd_pwm = int(np.clip(np.trunc(LR_RATIO * fwd_pwm), -255, 255))
    right_fwd_pwm = int(fwd_pwm)
    turn_pwm = int(np.clip(int(PWM_TURN), 0, 255))

    duration = DEFAULT_DURATION
    pan_angle = PAN_CENTER_DEG

    print("Interactive Robot Controls:")
    print("  f [sec]      -> forward")
    print("  b [sec]      -> backward")
    print("  l [sec]      -> turn left")
    print("  r [sec]      -> turn right")
    print("  p <angle>    -> pan servo to angle (0..180)")
    print("  pl [angle]   -> pan left (default -30 deg or given angle)")
    print("  pr [angle]   -> pan right (default +30 deg or given angle)")
    print("  pc           -> pan center (90 deg)")
    print("  read         -> read distance, omega_z, acceleration_y, and line sensors")
    print("  s            -> stop")
    print("  q            -> quit")
    print(f"Defaults: speed={abs(V_DRIVE):g} cm/s (PWM~{fwd_pwm}), turn PWM={turn_pwm}, duration={duration:g} s\n")

    with ElegooRobotCar(ROBOT_IP) as robot:
        try:
            robot.stop()
            robot.set_watchdog(1000)
            robot.pan(int(pan_angle))

            while True:
                try:
                    line = input(f"[dur={duration:g}s, pan={pan_angle}] cmd> ").strip().lower()
                except EOFError:
                    break
                except KeyboardInterrupt:
                    print()
                    robot.stop()
                    continue

                if not line:
                    continue

                parts = line.split()
                cmd = parts[0]

                if cmd in ("q", "quit", "exit"):
                    break
                if cmd in ("s", "stop"):
                    robot.stop()
                    print("Stopped.")
                    continue
                if cmd in ("read", "sensors", "d", "imu", "ir"):
                    print_sensors(robot)
                    continue

                # Pan commands
                if cmd in ("p", "pan"):
                    if len(parts) < 2:
                        print("Usage: p <angle_deg> (0..180)")
                        continue
                    try:
                        pan_angle = int(np.clip(float(parts[1]), 0, 180))
                        robot.pan(pan_angle)
                        print(f"Pan set to {pan_angle} deg.")
                    except ValueError:
                        print(f"Invalid pan angle: {parts[1]!r}")
                    continue

                if cmd in ("pl", "pan_left"):
                    try:
                        if len(parts) >= 2:
                            pan_angle = int(np.clip(float(parts[1]), 0, 180))
                        else:
                            pan_angle = int(np.clip(pan_angle - PAN_STEP_DEG, 0, 180))
                        robot.pan(pan_angle)
                        print(f"Pan left -> {pan_angle} deg.")
                    except ValueError:
                        print(f"Invalid pan angle: {parts[1]!r}")
                    continue

                if cmd in ("pr", "pan_right"):
                    try:
                        if len(parts) >= 2:
                            pan_angle = int(np.clip(float(parts[1]), 0, 180))
                        else:
                            pan_angle = int(np.clip(pan_angle + PAN_STEP_DEG, 0, 180))
                        robot.pan(pan_angle)
                        print(f"Pan right -> {pan_angle} deg.")
                    except ValueError:
                        print(f"Invalid pan angle: {parts[1]!r}")
                    continue

                if cmd in ("pc", "pan_center"):
                    pan_angle = PAN_CENTER_DEG
                    robot.pan(pan_angle)
                    print(f"Pan centered -> {pan_angle} deg.")
                    continue

                # Timed drive commands
                if len(parts) >= 2:
                    try:
                        duration = float(parts[1])
                        if duration < 0:
                            print("Duration must be non-negative.")
                            continue
                    except ValueError:
                        print(f"Invalid duration: {parts[1]!r}")
                        continue

                if cmd in ("f", "forward"):
                    run_timed_drive(robot, left_fwd_pwm, right_fwd_pwm, duration)
                elif cmd in ("b", "back", "backward"):
                    run_timed_drive(robot, -left_fwd_pwm, -right_fwd_pwm, duration)
                elif cmd in ("l", "left"):
                    # Flipped: (+turn_pwm, -turn_pwm) turns left on this chassis
                    run_timed_drive(robot, turn_pwm, -turn_pwm, duration)
                elif cmd in ("r", "right"):
                    # Flipped: (-turn_pwm, +turn_pwm) turns right on this chassis
                    run_timed_drive(robot, -turn_pwm, turn_pwm, duration)
                else:
                    print("Unknown command. Use f/b/l/r [sec], p <deg>, pl, pr, pc, read, s, or q.")

        finally:
            try:
                robot.stop()
                robot.pan(int(PAN_CENTER_DEG))
            except Exception:
                pass


if __name__ == "__main__":
    main()
