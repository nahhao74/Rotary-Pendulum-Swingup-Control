# Rotary Inverted Pendulum Modeling and Swing-Up Control

## 1. Project Overview

This project models and controls a rotary inverted pendulum system. The main objective is to swing the pendulum from the downward position to the upright region and then stabilize it around the upright equilibrium point.

The project contains two main parts:

1. **System modeling** using a linearized state-space model.
2. **Swing-up control** using a velocity-based resonant swing-up strategy.

---

## 2. Angle Convention and State Variables

The pendulum angle is defined as:

- `alpha = 0 rad`: pendulum at the upright equilibrium position.
- `alpha = pi rad`: pendulum at the downward position.

The rotary arm angle is denoted by `theta`.

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

where:

| Symbol | Meaning |
|---|---|
| `theta` | Rotary arm angle |
| `alpha` | Pendulum angle |
| `dot(theta)` | Rotary arm angular velocity |
| `dot(alpha)` | Pendulum angular velocity |

The control input is the motor voltage command:

```math
u = V_m
```

---

## 3. Physical Parameters

The physical parameters used in the model are:

```matlab
Rm = 8.4;       % Motor resistance [Ohm]
kt = 0.042;     % Motor torque constant [N.m/A]
km = 0.042;     % Back-EMF constant [V.s/rad]

m_rotor = 0.01; % Rotary arm mass [kg]
L_r = 0.085;    % Rotary arm length [m]

m_p = 0.024;    % Pendulum mass [kg]
L_p = 0.129;    % Pendulum length [m]
l_p = L_p / 2;  % Distance from pivot to pendulum center of mass [m]

g = 9.81;       % Gravitational acceleration [m/s^2]

J_m = 4.0e-6;   % Motor shaft inertia [kg.m^2]
J_h = 0.6e-6;   % Hub inertia [kg.m^2]

Dr = 0.0003;    % Rotary arm viscous damping
Dp = 0.00005;   % Pendulum viscous damping
```

---

## 4. Moment of Inertia Calculation

The pendulum moment of inertia about its center of mass is:

```math
J_{p,cm} = \frac{m_p L_p^2}{12}
```

Using the parallel-axis theorem, the pendulum moment of inertia about the pivot is:

```math
J_p = J_{p,cm} + m_p l_p^2
```

The rotary arm inertia is approximated as:

```math
J_{arm} = \frac{m_{rotor} L_r^2}{3}
```

The total equivalent rotary inertia is:

```math
J_r = J_m + J_h + J_{arm} + m_p L_r^2
```

In MATLAB form:

```matlab
J_p_cm = m_p * (L_p^2) / 12;
J_arm_rod = m_rotor * (L_r^2) / 3;

Jr_total = J_m + J_h + J_arm_rod + m_p * (L_r^2);
Jp_total = J_p_cm + m_p * (l_p^2);
```

---

## 5. Motor Model

The DC motor torque is modeled as:

```math
\tau_m = k_t i
```

The motor current is approximated by:

```math
i = \frac{V_m - k_m \dot{\theta}}{R_m}
```

Therefore:

```math
\tau_m =
\frac{k_t}{R_m}V_m
-
\frac{k_t k_m}{R_m}\dot{\theta}
```

The back-EMF term acts as an additional damping effect on the rotary arm:

```math
B_{emf} = \frac{k_t k_m}{R_m}
```

The total rotary damping is:

```math
D_{r,total} = D_r + B_{emf}
```

In MATLAB form:

```matlab
B_emf = (kt * km) / Rm;
Dr_total = Dr + B_emf;
```

---

## 6. Linearized State-Space Model

The linearized system is written as:

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

The determinant of the inertia matrix is:

```math
\Delta = J_r J_p - \left(m_p L_r l_p\right)^2
```

In MATLAB form:

```matlab
det_M = Jr_total * Jp_total - (m_p * L_r * l_p)^2;
```

The state matrix is:

```math
A =
\begin{bmatrix}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
0 & \frac{m_p^2 L_r^2 l_p g}{\Delta} &
-\frac{J_p D_{r,total}}{\Delta} &
-\frac{m_p L_r l_p D_p}{\Delta} \\
0 & \frac{m_p g l_p J_r}{\Delta} &
-\frac{m_p L_r l_p D_{r,total}}{\Delta} &
-\frac{J_r D_p}{\Delta}
\end{bmatrix}
```

The input matrix is:

```math
B =
\begin{bmatrix}
0 \\
0 \\
\frac{J_p(k_t/R_m)}{\Delta} \\
\frac{m_p L_r l_p(k_t/R_m)}{\Delta}
\end{bmatrix}
```

The MATLAB implementation is:

```matlab
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

## 7. Swing-Up Control Strategy

The implemented swing-up controller is a **velocity-based resonant swing-up controller**, not a full energy-based swing-up controller.

The control law used in the implementation is:

```c
float gain_reduction = 0.2f + 0.8f * fabs(sinf(alpha));

target_rpm = (-K_PUMP * gain_reduction * g_alpha_dot)
             - (K_CENTER * theta);
```

where:

| Symbol | Meaning |
|---|---|
| `alpha` | Pendulum angle |
| `g_alpha_dot` | Pendulum angular velocity, usually filtered |
| `theta` | Rotary arm angle |
| `K_PUMP` | Swing-up pumping gain |
| `K_CENTER` | Centering gain for the rotary arm |
| `target_rpm` | Motor speed command |

---

## 8. Interpretation of the Swing-Up Law

The term:

```math
-K_{PUMP} \cdot gain(\alpha) \cdot \dot{\alpha}
```

uses the pendulum angular velocity to create a resonant motion. This increases the oscillation amplitude and helps move the pendulum from the downward position toward the upright region.

The angle-dependent gain is:

```math
gain(\alpha) = 0.2 + 0.8|\sin(\alpha)|
```

This changes the pumping strength depending on the pendulum position.

The centering term is:

```math
-K_{CENTER}\theta
```

This term keeps the rotary arm near its reference position and prevents the arm from drifting too far during swing-up.

Therefore, the overall motor speed command can be written as:

```math
\omega_{ref}
=
-K_{PUMP}
\left(0.2 + 0.8|\sin(\alpha)|\right)
\dot{\alpha}
-
K_{CENTER}\theta
```

---

## 9. Switching to Balance Control

The swing-up controller is used only when the pendulum is far from the upright equilibrium point.

When the pendulum approaches the upright region, the controller should switch to a balance controller such as LQR combined with PID.

A typical switching condition can be:

```math
|\alpha| < \alpha_{sw}
```

and:

```math
|\dot{\alpha}| < \dot{\alpha}_{sw}
```

where:

| Symbol | Meaning |
|---|---|
| `alpha_sw` | Angle threshold for switching |
| `dot(alpha)_sw` | Angular velocity threshold for switching |

The general control process is:

```text
Downward position
        ↓
Velocity-based resonant swing-up
        ↓
Near upright region
        ↓
LQR + PID balance control
        ↓
Stable upright equilibrium
```

---

## 10. Difference from Energy-Based Swing-Up

A full energy-based swing-up method explicitly calculates the pendulum energy and compares it with the desired energy at the upright position.

For the convention:

```text
alpha = 0      upright position
alpha = pi     downward position
```

one possible energy expression is:

```math
E =
\frac{1}{2}J_p\dot{\alpha}^2
+
m_p g l_p(\cos\alpha - 1)
```

At the upright equilibrium:

```math
\alpha = 0, \quad \dot{\alpha}=0
```

therefore:

```math
E_{desired} = 0
```

The objective of energy-based swing-up is:

```math
E \rightarrow E_{desired}
```

In this project, the implemented controller does not directly calculate:

```math
E - E_{desired}
```

Instead, it uses pendulum angular velocity and angle-dependent gain to generate a resonant swing-up behavior.

---

## 11. Engineering Decision

A key engineering decision was to use a velocity-based resonant swing-up controller instead of a full energy-based swing-up controller.

This approach was selected because:

- It avoids more complex energy equations.
- It is easier to tune in simulation.
- It can bring the pendulum close to the upright region.
- It works well as a simple first swing-up strategy.

The limitation is that the swing-up time is not fully optimized. Future improvement can focus on implementing a full energy-based swing-up controller to reduce swing-up time and improve performance.

---

## 12. Summary

This project modeled a rotary inverted pendulum using a linearized state-space representation and implemented a velocity-based resonant swing-up controller. The model includes motor back-EMF damping, rotary arm inertia, pendulum inertia, viscous damping, and gravitational instability around the upright position.

The swing-up controller uses pendulum angular velocity to build oscillation energy and a centering term to keep the rotary arm near its reference position. Once the pendulum reaches the upright region, a balance controller such as LQR combined with PID can be used to stabilize the system.
