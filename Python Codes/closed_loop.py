import numpy as np
from  scipy.integrate import solve_ivp
from matplotlib import pyplot as plt

from motor_model import motor_ode
from pid_controller import PIDController

DT = 0.001
T_END = 10.0
SETPOINT_RPM = 100

def run_closed_loop(setpoint_rpm, Kp, Ki, Kd):
    pid = PIDController(Kp, Ki, Kd, dt=DT, output_max=12.0, output_min=-12.0)

    x = [0.0, 0.0]

    time_log, omega_log, voltage_log, error_log, current_log = [], [], [], [], []
    t = 0.0

    while t < T_END:
        omega_rpm = x[1] * 60 / (2 * np.pi)
        V = pid.compute(setpoint_rpm, omega_rpm)
        solution = solve_ivp(motor_ode, [t, t+DT], x, args=(V,), max_step=DT/10)
        x = solution.y[:, -1]
        time_log.append(t)
        omega_log.append(omega_rpm)
        voltage_log.append(V)
        error_log.append(setpoint_rpm - omega_rpm)
        current_log.append(x[0])
        t += DT

    return np.array(time_log), np.array(omega_log), np.array(voltage_log), np.array(error_log), np.array(current_log)

if __name__ == "__main__":
    t, omega, voltage, error, current = run_closed_loop(
        setpoint_rpm=SETPOINT_RPM,
        Kp=0.04, Ki=0.2, Kd=0.0
    )

    plt.figure(figsize=(12, 8))

    plt.subplot(2, 2, 1)
    plt.plot(t, omega)
    plt.axhline(SETPOINT_RPM, linestyle="--")
    plt.title("Speed Response")
    plt.xlabel("Time (s)")
    plt.ylabel("RPM")

    plt.subplot(2, 2, 2)
    plt.plot(t, voltage)
    plt.title("Control Voltage")
    plt.xlabel("Time (s)")
    plt.ylabel("Voltage (V)")

    plt.subplot(2, 2, 3)
    plt.plot(t, error)
    plt.title("Error")
    plt.xlabel("Time (s)")
    plt.ylabel("Error (RPM)")

    plt.subplot(2, 2, 4)
    plt.plot(t, current)
    plt.title("Control Current")
    plt.xlabel("Time (s)")
    plt.ylabel("Current (A)")

    plt.tight_layout()
    plt.show()
