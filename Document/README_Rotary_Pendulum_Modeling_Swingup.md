# Rotary Inverted Pendulum Modeling and Swing-Up Control

## 1. Overview

This document describes the modeling process and swing-up control strategy for a rotary inverted pendulum system. The system consists of a motor-driven rotary arm and a passive pendulum mounted at the end of the arm.

The main objective is:

- Model the rotary inverted pendulum as a state-space system.
- Use the model for controller design.
- Apply a velocity-based resonant swing-up controller to move the pendulum from the downward position to the upright region.

---

## 2. Coordinate Convention

The system uses two generalized coordinates:

| Symbol | Meaning |
|---|---|
| `theta` | Rotary arm angle |
| `alpha` | Pendulum angle |
| `dot(theta)` | Rotary arm angular velocity |
| `dot(alpha)` | Pendulum angular velocity |

The pendulum angle convention is:

| Pendulum position | Angle |
|---|---|
| Upright equilibrium position | `alpha = 0 rad` |
| Downward position | `alpha = pi rad` |

The state vector is defined as:

```math
x =
\begin{bmatrix}
\theta \\
\alpha \\
\dot{\theta} \\
\dot{\alpha}
\end{bmatrix}
```

The control input is the motor voltage command:

```math
u = V_m
```

The state-space form is:

```math
\dot{x} = Ax + Bu
```

---

## 3. Physical Parameters

The physical parameters used in the model are:

```matlab
Rm = 8.4;          % Motor resistance [Ohm]
kt = 0.042;        % Motor torque constant [N.m/A]
km = 0.042;        % Back-EMF constant [V.s/rad]

m_rotor = 0.01;    % Rotary arm mass [kg]
L_r = 0.085;       % Rotary arm length [m]

m_p = 0.024;       % Pendulum mass [kg]
L_p = 0.129;       % Pendulum length [m]
g = 9.81;          % Gravitational acceleration [m/s^2]

J_m = 4.0e-6;      % Motor shaft inertia [kg.m^2]
J_h = 0.6e-6;      % Hub inertia [kg.m^2]

l_p = L_p / 2;     % Distance from pendulum pivot to center of mass [m]

Dr = 0.0003;       % Rotary arm viscous damping
Dp = 0.00005;      % Pendulum viscous damping
```

---

## 4. Equivalent Moment of Inertia

The pendulum moment of inertia about its center of mass is:

```math
J_{p,cm} = \frac{m_p L_p^2}{12}
```

Using the parallel-axis theorem, the pendulum moment of inertia about the pivot is:

```math
J_p = J_{p,cm} + m_p l_p^2
```

The rotary arm is approximated as a slender rod rotating about one end:

```math
J_{arm} = \frac{m_{rotor}L_r^2}{3}
```

The total equivalent rotary inertia is:

```math
J_r = J_m + J_h + J_{arm} + m_pL_r^2
```

In MATLAB:

```matlab
J_p_cm = m_p * (L_p^2) / 12;
J_arm_rod = m_rotor * (L_r^2) / 3;

Jr_total = J_m + J_h + J_arm_rod + m_p * (L_r^2);
Jp_total = J_p_cm + m_p * (l_p^2);
```

where:

| Symbol | Meaning |
|---|---|
| `Jr_total` | Total equivalent inertia of the rotary arm side |
| `Jp_total` | Total equivalent inertia of the pendulum about its pivot |

---

## 5. Motor and Damping Model

The DC motor torque is:

```math
\tau_m = k_t i
```

The motor current is approximated as:

```math
i = \frac{V_m - k_m\dot{\theta}}{R_m}
```

Substituting the current into the torque equation:

```math
\tau_m =
\frac{k_t}{R_m}V_m
-
\frac{k_tk_m}{R_m}\dot{\theta}
```

The second term behaves like an electrical damping term due to back-EMF:

```math
B_{emf} = \frac{k_tk_m}{R_m}
```

Therefore, the total rotary damping is:

```math
D_{r,total} = D_r + B_{emf}
```

In MATLAB:

```matlab
B_emf = (kt * km) / Rm;
Dr_total = Dr + B_emf;
```

---

## 6. Inertia Matrix Determinant

The coupling term between the rotary arm and the pendulum is:

```math
m_pL_rl_p
```

The determinant of the coupled inertia matrix is:

```math
\Delta = J_rJ_p - (m_pL_rl_p)^2
```

In MATLAB:

```matlab
det_M = Jr_total * Jp_total - (m_p * L_r * l_p)^2;
```

This value appears in the denominator of the state-space model because the coupled equations of motion must be solved for:

```math
\ddot{\theta}
```

and:

```math
\ddot{\alpha}
```

---

## 7. Linearized State-Space Model

The state-space model is written as:

```math
\dot{x} = Ax + Bu
```

with:

```math
x =
\begin{bmatrix}
\theta \\
\alpha \\
\dot{\theta} \\
\dot{\alpha}
\end{bmatrix}
```

The state matrix is:

```math
A =
\begin{bmatrix}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
0 & \frac{m_p^2L_r^2l_pg}{\Delta} &
-\frac{J_pD_{r,total}}{\Delta} &
-\frac{m_pL_rl_pD_p}{\Delta} \\
0 & \frac{m_pgl_pJ_r}{\Delta} &
-\frac{m_pL_rl_pD_{r,total}}{\Delta} &
-\frac{J_rD_p}{\Delta}
\end{bmatrix}
```

The input matrix is:

```math
B =
\begin{bmatrix}
0 \\
0 \\
\frac{J_p(k_t/R_m)}{\Delta} \\
\frac{m_pL_rl_p(k_t/R_m)}{\Delta}
\end{bmatrix}
```

The first two rows of `A` represent the kinematic relationship:

```math
\dot{\theta} = \dot{\theta}
```

```math
\dot{\alpha} = \dot{\alpha}
```

The last two rows represent the angular accelerations of the rotary arm and the pendulum.

The term related to `alpha` represents the effect of gravity around the upright equilibrium position. The damping terms are included through `Dr_total` and `Dp`.

---

## 8. MATLAB Implementation of the Model

```matlab
%% Physical parameters
Rm = 8.4;
kt = 0.042;
km = 0.042;

m_rotor = 0.01;
L_r = 0.085;

m_p = 0.024;
L_p = 0.129;
g = 9.81;

J_m = 4.0e-6;
J_h = 0.6e-6;
l_p = L_p / 2;

J_p_cm = m_p * (L_p^2) / 12;
J_arm_rod = m_rotor * (L_r^2) / 3;

Jr_total = J_m + J_h + J_arm_rod + m_p * (L_r^2);
Jp_total = J_p_cm + m_p * (l_p^2);

Dr = 0.0003;
Dp = 0.00005;

det_M = Jr_total * Jp_total - (m_p * L_r * l_p)^2;

B_emf = (kt * km) / Rm;
Dr_total = Dr + B_emf;

%% State-space model
A = zeros(4,4);

A(1,3) = 1;
A(2,4) = 1;

A(3,2) = (m_p^2 * L_r^2 * l_p * g) / det_M;
A(3,3) = -(Jp_total * Dr_total) / det_M;
A(3,4) = -(m_p * L_r * l_p * Dp) / det_M;

A(4,2) = (m_p * g * l_p * Jr_total) / det_M;
A(4,3) = -(m_p * L_r * l_p * Dr_total) / det_M;
A(4,4) = -(Jr_total * Dp) / det_M;

B = zeros(4,1);

B(3) = (Jp_total * (kt / Rm)) / det_M;
B(4) = (m_p * L_r * l_p * (kt / Rm)) / det_M;
```

---

## 9. Swing-Up Control Objective

When the pendulum starts from the downward position:

```math
\alpha \approx \pi
```

the balance controller cannot directly stabilize it at the upright point because the pendulum is too far from the linearized operating region.

Therefore, the swing-up controller is used first.

The objective of the swing-up controller is to increase the pendulum oscillation amplitude until the pendulum approaches:

```math
\alpha \approx 0
```

After that, the system can switch to the upright balance controller.

---

## 10. Velocity-Based Resonant Swing-Up Law

The implemented swing-up law is:

```c
float gain_reduction = 0.2f + 0.8f * fabs(sinf(alpha));

target_rpm = (-K_PUMP * gain_reduction * g_alpha_dot)
             - (K_CENTER * theta);
```

In mathematical form:

```math
\omega_{ref}
=
-K_{PUMP}
\left(0.2 + 0.8|\sin(\alpha)|\right)
\dot{\alpha}
-
K_{CENTER}\theta
```

where:

| Symbol | Meaning |
|---|---|
| `omega_ref` | Motor speed reference |
| `K_PUMP` | Swing-up pumping gain |
| `K_CENTER` | Rotary arm centering gain |
| `alpha` | Pendulum angle |
| `dot(alpha)` | Pendulum angular velocity |
| `theta` | Rotary arm angle |

---

## 11. Meaning of Each Swing-Up Term

### 11.1 Pumping Term

```math
-K_{PUMP}
\left(0.2 + 0.8|\sin(\alpha)|\right)
\dot{\alpha}
```

This term uses the pendulum angular velocity to command the motor speed. The motor motion is generated based on the direction and speed of the pendulum motion.

The purpose is to excite the pendulum motion and increase its oscillation amplitude until it reaches the upright region.

The sign of this term depends on:

- Motor rotation direction
- Encoder direction
- Definition of positive `alpha`
- Definition of positive `theta`

If the pendulum loses energy or the swing amplitude decreases, the sign of `K_PUMP` or the measured angular velocity direction should be checked.

### 11.2 Angle-Dependent Gain

```math
0.2 + 0.8|\sin(\alpha)|
```

This gain changes the pumping strength based on pendulum angle.

Its value is bounded by:

```math
0.2 \leq gain(\alpha) \leq 1.0
```

When:

```math
|\sin(\alpha)| = 0
```

the gain becomes:

```math
gain(\alpha) = 0.2
```

When:

```math
|\sin(\alpha)| = 1
```

the gain becomes:

```math
gain(\alpha) = 1.0
```

This means the controller applies weaker pumping near the vertical positions and stronger pumping around the middle region of the swing.

### 11.3 Centering Term

```math
-K_{CENTER}\theta
```

This term pulls the rotary arm back toward the center position:

```math
\theta = 0
```

Without this term, the rotary arm may drift too far while trying to swing the pendulum up.

The centering term helps keep the system within a usable operating range.

---

## 12. Swing-Up Algorithm

The swing-up procedure can be written as:

```text
1. Measure pendulum angle alpha.
2. Measure or estimate pendulum angular velocity dot(alpha).
3. Measure rotary arm angle theta.
4. Calculate angle-dependent gain:
       gain = 0.2 + 0.8 * abs(sin(alpha))
5. Calculate motor speed reference:
       target_rpm = -K_PUMP * gain * dot(alpha) - K_CENTER * theta
6. Apply speed command to the motor controller.
7. Repeat until the pendulum reaches the upright region.
```

A simple implementation structure is:

```c
if (fabs(alpha) > alpha_switch)
{
    float gain_reduction = 0.2f + 0.8f * fabs(sinf(alpha));

    target_rpm = (-K_PUMP * gain_reduction * g_alpha_dot)
                 - (K_CENTER * theta);
}
else
{
    // Switch to upright balance controller
}
```

---

## 13. Switching Condition

The swing-up controller should operate when the pendulum is far from the upright position.

When the pendulum is close enough to upright, the controller should switch to a balance controller.

A common switching condition is:

```math
|\alpha| < \alpha_{sw}
```

and optionally:

```math
|\dot{\alpha}| < \dot{\alpha}_{sw}
```

where:

| Symbol | Meaning |
|---|---|
| `alpha_sw` | Angle threshold for switching |
| `dot(alpha)_sw` | Angular velocity threshold for switching |

The second condition helps avoid switching when the pendulum is close to upright but moving too fast.

---

## 14. Tuning Notes

### 14.1 `K_PUMP`

`K_PUMP` controls how strongly the system excites the pendulum.

If `K_PUMP` is too small:

- The pendulum may not swing up.
- The swing-up time becomes long.

If `K_PUMP` is too large:

- The rotary arm may move aggressively.
- The pendulum may overshoot the upright region.
- The motor command may saturate.

### 14.2 `K_CENTER`

`K_CENTER` controls how strongly the rotary arm is pulled back to the center.

If `K_CENTER` is too small:

- The rotary arm may drift away from the center.

If `K_CENTER` is too large:

- The centering action may fight against the swing-up motion.
- The pendulum may not gain enough oscillation amplitude.

### 14.3 `alpha_switch`

`alpha_switch` defines when to change from swing-up mode to balance mode.

If `alpha_switch` is too large:

- The balance controller may activate too early.

If `alpha_switch` is too small:

- The system may miss the switching window.

---

## 15. Summary

The rotary inverted pendulum was modeled using a linearized state-space representation with the state vector:

```math
x =
\begin{bmatrix}
\theta &
\alpha &
\dot{\theta} &
\dot{\alpha}
\end{bmatrix}^T
```

The model includes:

- Rotary arm inertia
- Pendulum inertia
- Motor torque constant
- Back-EMF damping
- Viscous damping
- Gravity effect
- Coupling between rotary arm and pendulum

The swing-up controller uses the pendulum angular velocity, an angle-dependent gain, and a rotary arm centering term to generate a motor speed reference. This allows the system to build pendulum oscillation and move toward the upright region before switching to the balance controller.
