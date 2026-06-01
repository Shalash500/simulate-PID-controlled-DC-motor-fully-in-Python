import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Motor parameters
R = 5.0         # Armature resistance
L = 0.01        # Armature inductance
Ke = 0.012      # Back-EMF constant
Kt = 0.012      # Torque constant
J = 5e-5        # Rotor inertia
b = 1e-4        # Viscous friction
TL = 0          # Load torque (free run)

def motor_ode(t, x, V_input):
    I, omega = x
    dI_dt = (V_input - R*I - Ke * omega) / L
    domega_dt = (Kt * I - b * omega - TL) / J

    return [dI_dt, domega_dt]

def run_constant_voltage(V_input):
    V = V_input
    time_span = [0, 2]
    x0 = [0.0, 0.0]
    t_eval = np.linspace(0, 2, 1000)

    solution = solve_ivp(motor_ode, time_span, x0, t_eval=t_eval, args=(V,))

    I = solution.y[0]
    omega = solution.y[1]

    plt.figure(figsize=(10, 4))

    plt.subplot(1, 2, 1)
    plt.plot(solution.t, omega * 60 / (2 * np.pi))
    plt.title("Speed (RPM)")
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (RPM)")

    plt.subplot(1, 2, 2)
    plt.plot(solution.t, I)
    plt.title("Current (A)")
    plt.xlabel("Time (s)")
    plt.ylabel("Current (A)")

    plt.tight_layout()
    plt.show()

def run_voltage_switching(v1=12, v2=6):
    x0 = [0.0, 0.0]
    t_eval_1 = np.linspace(0, 1, 1000)
    t_eval_2 = np.linspace(1, 2, 1000)
    solution_1 = solve_ivp(motor_ode, [0, 1], x0, t_eval=t_eval_1, args=(v1,))
    I_1 = solution_1.y[0]
    omega_1 = solution_1.y[1]
    x1 = solution_1.y[:, -1]

    solution_2 = solve_ivp(motor_ode, [1, 2], x1, t_eval=t_eval_2, args=(v2,))
    I_2 = solution_2.y[0]
    omega_2 = solution_2.y[1]

    I = np.concatenate([I_1, I_2])
    omega = np.concatenate([omega_1, omega_2])

    t = np.concatenate([solution_1.t, solution_2.t])

    plt.figure(figsize=(10, 4))

    plt.subplot(1, 2, 1)
    plt.plot(t, omega * 60 / (2 * np.pi))
    plt.title("Speed (RPM)")
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (RPM)")

    plt.subplot(1, 2, 2)
    plt.plot(t, I)
    plt.title("Current (A)")
    plt.xlabel("Time (s)")
    plt.ylabel("Current (A)")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_constant_voltage(12)
    run_voltage_switching(12, 6)
    run_voltage_switching(12, 0)    # coast-down running
