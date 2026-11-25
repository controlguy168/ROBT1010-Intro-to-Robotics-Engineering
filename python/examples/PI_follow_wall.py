import time
import numpy as np
import matplotlib.pyplot as plt

from client import ElegooRobotCar


# ---------------------------------------------------------------------------
# Adjustable Parameters
# ---------------------------------------------------------------------------
ROBOT_IP = "192.168.4.1"

V_SET = 15.0          # Forward/backward linear velocity (cm/s); set to 50.0 when tuned
D_TARGET = 50.0       # Desired distance from wall (cm)
RUN_DURATION = 15.0   # Total run time (sec)
T_S = 0.1             # Control loop sampling period (sec)

WALL_SIDE = "left"    # "left" or "right"
PAN_LEFT_DEG = 0      # Adjust to whichever angle (0 or 180) faces the LEFT wall
PAN_RIGHT_DEG = 180   # Adjust to whichever angle (0 or 180) faces the RIGHT wall
PAN_CENTER_DEG = 95   # Forward-facing center angle

# Flip this to True if omega steers toward the wall instead of away from it
REVERSE_STEERING = False

# PI Controller Gains for angular velocity omega (rad/s)
K_P = 0.2            # Proportional gain (rad/s per cm)
K_I = 0.01           # Integral gain (rad/s per cm*s)
OMEGA_MAX = 2.0       # Max angular velocity magnitude (rad/s)
I_MAX = 40.0          # Integral windup limit (cm*s)

# Robot Geometry, Filtering & Safety
WHEEL_BASE = 14.0     # Effective track width L between left/right wheels (cm)
D_MIN_VALID = 2.0     # Minimum plausible ultrasonic reading (cm)
D_MAX_VALID = 250.0   # Maximum plausible ultrasonic reading (cm)
D_JUMP_THRESH = 30.0  # Reject single-step jumps larger than this (cm)
D_CRASH_STOP = 10.0   # Emergency stop if wall gets closer than this (cm)


def extract_distance_cm(reading: object) -> float:
    """Extract distance (cm) from the JSON dict returned by read_distance()."""
    if isinstance(reading, (int, float)) and not isinstance(reading, bool):
        return float(reading)
    if isinstance(reading, dict):
        for key in ("distance", "dist", "cm", "range", "value"):
            if key in reading and isinstance(reading[key], (int, float)) and not isinstance(reading[key], bool):
                return float(reading[key])
        for val in reading.values():
            if isinstance(val, (int, float)) and not isinstance(val, bool):
                return float(val)
    raise ValueError(f"Unrecognized distance reading format: {reading!r}")


def v2pwm(v: float) -> int:
    """Map wheel linear velocity (cm/s) to integer motor PWM in [-255, 255]."""
    if abs(v) < 1e-6:
        return 0
    pwm_val = int(np.trunc(3.0207 * v - 1.73)) if v > 0 else int(np.trunc(3.0207 * v + 1.73))
    return int(np.clip(pwm_val, -255, 255))


def main() -> None:
    if WALL_SIDE.lower() == "left":
        pan_angle = PAN_LEFT_DEG
        # d > D_TARGET (too far from left wall) -> need +omega (turn left toward wall)
        # d < D_TARGET (too close to left wall) -> need -omega (turn right away from wall)
        side_sign = 1.0
    elif WALL_SIDE.lower() == "right":
        pan_angle = PAN_RIGHT_DEG
        # d > D_TARGET (too far from right wall) -> need -omega (turn right toward wall)
        # d < D_TARGET (too close to right wall) -> need +omega (turn left away from wall)
        side_sign = -1.0
    else:
        raise ValueError("WALL_SIDE must be 'left' or 'right'")

    if REVERSE_STEERING:
        side_sign *= -1.0

    t_log = []
    d_raw_log = []
    d_filt_log = []
    err_log = []
    omega_log = []
    pwm_l_log = []
    pwm_r_log = []

    with ElegooRobotCar(ROBOT_IP) as robot:
        try:
            robot.stop()
            robot.set_watchdog(1000)

            robot.pan(int(pan_angle))
            time.sleep(0.5)

            d_prev = None
            err_int = 0.0
            t_start = time.perf_counter()

            while True:
                now = time.perf_counter()
                elapsed_total = now - t_start
                if elapsed_total >= RUN_DURATION:
                    break

                # 1. Read and filter wall distance
                d_raw = extract_distance_cm(robot.read_distance())
                if not (D_MIN_VALID <= d_raw <= D_MAX_VALID):
                    d = d_prev if d_prev is not None else D_TARGET
                elif d_prev is not None and abs(d_raw - d_prev) > D_JUMP_THRESH:
                    d = d_prev
                else:
                    d = d_raw
                d_prev = d

                # Safety stop if too close to wall
                if d < D_CRASH_STOP:
                    robot.stop()
                    print(f"Emergency stop: distance {d:.2f} cm < {D_CRASH_STOP:.2f} cm")
                    break

                # 2. PI controller on omega
                err = side_sign * (d - D_TARGET)
                err_int = float(np.clip(err_int + err * T_S, -I_MAX, I_MAX))

                omega = K_P * err + K_I * err_int
                omega = float(np.clip(omega, -OMEGA_MAX, OMEGA_MAX))

                # 3. Differential-drive kinematics -> left/right wheel speeds & PWMs
                v_left = V_SET - 0.5 * WHEEL_BASE * omega
                v_right = V_SET + 0.5 * WHEEL_BASE * omega

                pwm_left = int(v2pwm(v_left))
                pwm_right = int(v2pwm(v_right))

                # 4. Drive for sampling period T_S
                step_start = time.perf_counter()
                while (time.perf_counter() - step_start) < T_S:
                    if (time.perf_counter() - t_start) >= RUN_DURATION:
                        break
                    robot.drive(pwm_left, pwm_right)
                    time.sleep(0.01)

                t_log.append(elapsed_total)
                d_raw_log.append(d_raw)
                d_filt_log.append(d)
                err_log.append(err)
                omega_log.append(omega)
                pwm_l_log.append(pwm_left)
                pwm_r_log.append(pwm_right)

                print(
                    f"t={elapsed_total:5.2f}s | d={d:6.2f} (raw={d_raw:6.2f}) | "
                    f"err={err:+6.2f} | omega={omega:+6.3f} | PWM=({pwm_left:+4d}, {pwm_right:+4d})"
                )

        finally:
            try:
                robot.stop()
                robot.pan(int(PAN_CENTER_DEG))
            except Exception:
                pass

    t_arr = np.array(t_log)
    d_raw_arr = np.array(d_raw_log)
    d_filt_arr = np.array(d_filt_log)
    omega_arr = np.array(omega_log)
    pwm_l_arr = np.array(pwm_l_log)
    pwm_r_arr = np.array(pwm_r_log)

    if len(t_arr) > 0:
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))

        ax1.plot(t_arr, d_raw_arr, "o--", alpha=0.5, label="raw distance")
        ax1.plot(t_arr, d_filt_arr, "o-", linewidth=1.5, label="filtered distance")
        ax1.axhline(D_TARGET, color="r", linestyle=":", linewidth=1.5, label="target distance")
        ax1.set_ylabel("Wall Distance (cm)")
        ax1.set_title(f"Wall Following (v={V_SET:g} cm/s, kp={K_P:g}, ki={K_I:g})")
        ax1.grid(True)
        ax1.legend()

        ax2.plot(t_arr, omega_arr, "s-", linewidth=1.5, label="omega (rad/s)")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Angular Velocity (rad/s)")
        ax2.grid(True)
        ax2.legend()

        plt.tight_layout()

        savefile = f"wall_follow_v{V_SET:g}_d{D_TARGET:g}_kp{K_P:g}_ki{K_I:g}.npz"
        np.savez(
            savefile,
            t=t_arr,
            d_raw=d_raw_arr,
            d_filt=d_filt_arr,
            err=np.array(err_log),
            omega=omega_arr,
            pwm_left=pwm_l_arr,
            pwm_right=pwm_r_arr,
            v_set=V_SET,
            d_target=D_TARGET,
            k_p=K_P,
            k_i=K_I,
        )
        print(f"Saved run data to {savefile}")
        plt.show()


if __name__ == "__main__":
    main()
