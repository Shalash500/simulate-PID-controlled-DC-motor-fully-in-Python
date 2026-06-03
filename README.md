# PID-Controlled DC Motor Simulation in Python

Closed-loop simulation of a DC motor with PID speed control, implemented from first principles in Python and validated with a parallel Simulink model.

---

## What this project does

- Derives the DC motor differential equations from first principles (KVL + Newton's 2nd law for rotation)
- Implements the motor as a system of two coupled first-order ODEs solved with `scipy.integrate.solve_ivp`
- Implements a discrete PID controller class with conditional integration anti-windup
- Connects controller and plant in a closed-loop time-step simulation
- Provides an interactive Matplotlib slider widget for live gain tuning
- Reproduces the same closed-loop system in Simulink for cross-tool verification

---

## Motor model

The DC motor is described by two coupled ODEs derived from the armature circuit (KVL) and shaft dynamics (Newton's 2nd law for rotation):

```
dI/dt   = (V - R·I - Ke·ω) / L
dω/dt   = (Kt·I - b·ω - TL) / J
```

State vector: `x = [I, ω]` — armature current and angular velocity.  
Input: `V` — applied voltage from the PID controller.

### Motor parameters

| Parameter | Symbol | Value | Unit |
|---|---|---|---|
| Armature resistance | R | 5.0 | Ω |
| Armature inductance | L | 0.01 | H |
| Back-EMF constant | Ke | 0.012 | V·s/rad |
| Torque constant | Kt | 0.012 | N·m/A |
| Rotor inertia | J | 5×10⁻⁵ | kg·m² |
| Viscous friction | b | 1×10⁻⁴ | N·m·s/rad |
| Load torque | TL | 0 | N·m |

Parameters are physically motivated estimates for a small 12V DC gearmotor. No datasheet was available — J and b are intended to be refined against hardware data in a future update.

**Note on Ke = Kt:** In SI units these are numerically equal by conservation of energy. This is a property of an ideal DC motor, not an assumption.

---

## PID controller

The `PIDController` class implements a discrete PID algorithm with **conditional integration anti-windup**: the integral term only accumulates when the output is not saturated. This prevents integrator windup during the startup transient when the controller is voltage-limited.

```python
pid = PIDController(Kp=0.04, Ki=0.2, Kd=0.0, dt=0.001,
                    output_min=-12.0, output_max=12.0)
```

Initial gains were found to cause instability in simulation. This is because the simulated motor's electrical time constant (τ_e = L/R = 2ms) requires a fast sample rate to remain stable. DT=1ms was used and gains were tuned accordingly.

---

## Closed-loop simulation results

**Python simulation** — setpoint 100 RPM, Kp=0.04, Ki=0.2, Kd=0.0, DT=1ms:

![PID controlled closed loop](output%20figures/PID%20controlled%20closed%20loop.png)

- Fast rise, small overshoot (~5 RPM), settles at exactly 100 RPM
- Zero steady-state error — integral term working correctly
- Voltage saturates at startup then drops to ~0.55V at steady state

**Interactive slider tuner:**

![PID Tuner](output%20figures/PID%20Tuner.png)

---

## Open-loop verification

Three open-loop tests validate the motor ODE before closing the loop:

| Test | Description |
|---|---|
| Constant 12V | Motor accelerates to no-load speed, current decays as back-EMF builds |
| Step down 12V → 6V | Speed drops to new steady state, current drops sharply then recovers |
| Coast down 12V → 0V | Speed decays under friction; negative current visible (regenerative braking in full model) |

![Constant voltage open loop](output%20figures/Constant%20voltage%20open%20loop.png)

---

## Simulink validation

The same DC motor ODE and closed-loop PID structure were reproduced independently in MATLAB/Simulink.

**Open-loop model** — armature circuit and mechanical subsystem built from integrator blocks:

![Simulink open loop](Simulink/Simulink%20open%20loop.png)

Open-loop outputs match Python simulation:
- Steady-state omega: ~225 rad/s (Simulink) vs ~2150 RPM = 225 rad/s (Python) ✅
- Steady-state current: ~1.87A in both ✅

![Simulink open loop omega](Simulink/Simulink%20open%20loop%20omega%20output.png)

**Closed-loop model** — PID Controller block (continuous-time, parallel form) with voltage saturation:

![Simulink closed loop](Simulink/Simulink%20closed%20loop.png)

Closed-loop step response settles at setpoint (100 rad/s) with small overshoot, consistent with Python results:

![Simulink closed loop output](Simulink/Simulink%20closed%20loop%20output.png)

**Note on PID implementation difference:** The Python PID is discrete (DT=1ms, conditional integration anti-windup). The Simulink PID block is continuous-time with built-in anti-windup. Both produce stable responses with the same qualitative behavior. A direct quantitative comparison would require switching the Simulink PID to discrete mode with matched sample time.

---

## Repository structure

```
├── Python Codes/
│   ├── motor_model.py       — DC motor ODE + open-loop tests
│   ├── pid_controller.py    — PID class with anti-windup
│   ├── closed_loop.py       — closed-loop simulation + 2×2 plot
│   └── slider_tuner.py      — interactive Matplotlib gain tuner
├── Simulink/
│   ├── DC_Motor_model_open_loop.slx
│   ├── DC_Motor_model_closed_loop.slx
│   └── *.png                — scope screenshots
└── output figures/
    └── *.png                — Python simulation plots
```

---

## How to run

**Requirements:**
```
pip install numpy scipy matplotlib
```

**Open-loop motor model:**
```bash
cd "Python Codes"
python motor_model.py
```

**Closed-loop PID simulation:**
```bash
python closed_loop.py
```

**Interactive slider tuner:**
```bash
python slider_tuner.py
```

---

## What I learned

- Deriving a physical system's ODEs from circuit laws and Newton's 2nd law, then implementing them directly in code
- How the electrical time constant (τ_e = L/R) constrains the minimum stable controller sample rate
- Discrete PID implementation with conditional integration anti-windup — why back-calculation anti-windup can produce sign-reversal instability
- The difference between the gains that stabilize real hardware and those required for a simulation with faster dynamics — and what that gap reveals about model accuracy
- Cross-tool verification: building the same model independently in Python (SciPy) and MATLAB/Simulink and comparing outputs