import time
import numpy as np
import matplotlib.pyplot as plt

from client import ElegooRobotCar


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
    pwm_val = int(np.trunc(3.0207 * v - 1.73))
    return int(np.clip(pwm_val, -255, 255))


def main() -> None:
    # Set constants
    k_max = 50        # maximum # of iterations
    k_p = 5.0         # proportional feedback gain
    k_i = 1.0         # integral feedback gain
    t_s = 0.2         # assumed sampling period (sec)
    x_threshold = 5.0 # detection region of spurious reading
    u_max = 25.0      # velocity bound in [-u_max, u_max] (cm/s)
    alpha = 0.5       # trust of prediction vs. measurement when they are close
    a = 1.0           # L/R wheel speed ratio to keep a straight line

    x = np.zeros(k_max, dtype=float)
    x_pred = np.zeros(k_max, dtype=float)
    xI = np.zeros(k_max, dtype=float)
    u = np.zeros(k_max, dtype=float)
    u_est = np.zeros(k_max, dtype=float)
    pwm = np.zeros(k_max, dtype=int)
    dt = np.zeros(k_max, dtype=float)

    with ElegooRobotCar("192.168.4.1") as robot:
        try:
            robot.stop()
            robot.set_watchdog(1000)

            raw_target = input("enter target distance (cm): ")
            xstar = float(np.clip(float(raw_target), 50.0, 200.0))

            # Control loop
            for k in range(k_max):
                # Nominal range sensor reading
                x[k] = extract_distance_cm(robot.read_distance())

                # Correction based on model prediction
                if k > 0:
                    x_pred_k = x_pred[k - 1] - u[k - 1] * dt[k - 1]
                    u_est[k - 1] = -(x[k] - x[k - 1]) / dt[k - 1]

                    if abs(x_pred_k - x[k]) < x_threshold:
                        x_pred_k = alpha * x_pred_k + (1.0 - alpha) * x[k]

                    x_pred[k] = x_pred_k
                    xI[k] = xI[k - 1] + x_pred[k] - xstar
                else:
                    x_pred[k] = x[k]
                    xI[k] = 0.0

                u_k = (k_p * (x_pred[k] - xstar) + k_i * xI[k]) * t_s
                u[k] = float(np.clip(u_k, -u_max, u_max))

                if abs(x_pred[k] - xstar) < 2.0 or (k > 0 and u[k - 1] == 0.0):
                    u[k] = 0.0

                left_pwm = int(v2pwm(u[k]))
                right_pwm = int(np.clip(np.trunc(a * left_pwm), -255, 255))
                pwm[k] = left_pwm

                # Timed drive interval
                timer_start = time.perf_counter()
                elapsed = 0.0
                while elapsed < t_s:
                    robot.drive(int(left_pwm), int(right_pwm))
                    elapsed = time.perf_counter() - timer_start
                dt[k] = elapsed

                robot.stop()

                print(
                    f"k= {k + 1}, x={x[k]:g}, x_pred={x_pred[k]:g}, "
                    f"u={u[k]:g}, PWM={left_pwm}, dt ={dt[k]:g}"
                )

        finally:
            try:
                robot.stop()
            except Exception:
                pass

    t = np.cumsum(dt)
    u_est[-1] = u[-1]

    # Plot 1: Commanded vs. estimated velocity
    plt.figure()
    plt.plot(t, u, "o-", linewidth=1.5, label="u")
    plt.plot(t, u_est, "x-", linewidth=1.5, label="estimated u based on y")
    plt.grid(True)
    plt.xlabel("time (sec)")
    plt.ylabel("velocity (cm/s)")
    plt.title(f"PI Controller, kp={k_p:g}, ki={k_i:g}")
    plt.legend()

    # Plot 2: Range sensor, prediction, and target
    plt.figure()
    plt.plot(t, x, "o-", linewidth=1.5, label="sensor reading")
    plt.plot(t, x_pred, "x-", linewidth=1.5, label="prediction")
    plt.plot(t, np.full_like(t, xstar), ":", linewidth=1.5, label="target")
    plt.grid(True)
    plt.xlabel("time (sec)")
    plt.ylabel("range sensor output (cm)")
    plt.title(f"PI Controller, kp={k_p:g}, ki={k_i:g}")
    plt.legend()

    savefile = (
        f"PI_with_prediction_kp{k_p:g}_ki{k_i:g}_"
        f"xth{x_threshold:g}_xstar_{xstar:g}.npz"
    )
    np.savez(
        savefile,
        k_p=k_p,
        k_i=k_i,
        t_s=t_s,
        x_threshold=x_threshold,
        t=t,
        u=u,
        u_est=u_est,
        xstar=xstar,
        x_pred=x_pred,
        x=x,
    )
    print(f"Saved data to {savefile}")

    plt.show()


if __name__ == "__main__":
    main()
