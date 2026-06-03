import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

from closed_loop import run_closed_loop

SETPOINT_RPM = 100

Kp0 = 0.05
Ki0 = 0.0
Kd0 = 0.0

t, omega, _, _, _ = run_closed_loop(
    SETPOINT_RPM,
    Kp0,
    Ki0,
    Kd0
)

fig, ax = plt.subplots(figsize=(10, 6))
plt.subplots_adjust(bottom=0.35)

line, = ax.plot(t, omega, linewidth=2)

ax.axhline(
    SETPOINT_RPM,
    linestyle='--',
    color='r',
    label='Setpoint'
)

ax.set_title("PID Speed Control Tuning")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Speed (RPM)")
ax.grid(True)
ax.legend()

ax_kp = plt.axes((0.15, 0.20, 0.70, 0.03))
ax_ki = plt.axes((0.15, 0.13, 0.70, 0.03))
ax_kd = plt.axes((0.15, 0.06, 0.70, 0.03))

slider_kp = Slider(
    ax=ax_kp,
    label='Kp',
    valmin=0.0,
    valmax=0.2,
    valinit=Kp0
)

slider_ki = Slider(
    ax=ax_ki,
    label='Ki',
    valmin=0.0,
    valmax=1.0,
    valinit=Ki0
)

slider_kd = Slider(
    ax=ax_kd,
    label='Kd',
    valmin=0.0,
    valmax=0.01,
    valinit=Kd0
)

def update(val):

    kp = slider_kp.val
    ki = slider_ki.val
    kd = slider_kd.val

    t, omega, _, _, _ = run_closed_loop(
        SETPOINT_RPM,
        kp,
        ki,
        kd
    )

    line.set_xdata(t)
    line.set_ydata(omega)

    ax.relim()
    ax.autoscale_view()

    plt.draw()


slider_kp.on_changed(update)
slider_ki.on_changed(update)
slider_kd.on_changed(update)

if __name__ == "__main__":
    plt.show()