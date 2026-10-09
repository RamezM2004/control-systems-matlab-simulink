# Classical Control Systems Engineering: Dynamic Modeling, Root Locus & PID/PD Design
### MATLAB & Simulink Analysis of Electromechanical Plants and Frequency-Domain Compensation

**Course:** ME0344 – Control Systems I  
**Authors:** Ramez Al-Masadeh & Abdullah Al-Bakri  
**Institution:** German Jordanian University (GJU)  
**Full Report:** [`docs/Control_Systems_1_Project_Report.pdf`](docs/Control_Systems_1_Project_Report.pdf) (12 Pages)

---

## Project Overview

This repository contains the complete analytical solutions, MATLAB simulation scripts, and root locus design procedures for the **ME0344 Control Systems I Final Capstone Project**.

The project is divided into two core engineering problems:
1. **Question 1: Armature-Controlled DC Motor Position Servo**: Mathematical modeling from motor torque-speed characteristics, open-loop vs. closed-loop step response analysis, and PID controller tuning for $>20\%$ settling time reduction with zero steady-state error.
2. **Question 2: Third-Order Plant Compensation via Root Locus**: Theoretical analysis of why proportional control alone cannot achieve aggressive settling time specifications ($T_s < 2.07\text{ s}$), followed by PD zero placement to reshape root locus branches for $\tau \le 0.5\text{ s}$ and $\zeta \ge 0.707$.

---

## Part 1: Armature-Controlled DC Motor (`Control_Q1_DCMotor_Step_PID.m`)

### Mathematical Model
From the torque-speed envelope and Kirchhoff’s voltage law:
$$e_a(t) = R_a i_a(t) + L_a \frac{di_a(t)}{dt} + e_b(t), \quad e_b(t) = K_b \dot{\theta}_m(t)$$
$$T_m(t) = K_t i_a(t) = J \ddot{\theta}_m(t) + D \dot{\theta}_m(t)$$

Neglecting armature inductance ($L_a \approx 0$), the open-loop transfer function relating armature voltage $E_a(s)$ to shaft angular position $\theta_m(s)$ is:
$$G(s) = \frac{\Theta_m(s)}{E_a(s)} = \frac{K_t}{s \left[ (R_a J)s + (R_a D + K_t K_b) \right]}$$

### Performance Comparison: Uncompensated vs. PID Compensated
| Performance Metric | Open-Loop | Closed-Loop (Unity Feedback) | PID Tuned Closed-Loop | Design Requirement |
| :--- | :---: | :---: | :---: | :---: |
| **Rise Time ($T_r$)** | N/A (Unbounded) | $3.4314\text{ s}$ | $\mathbf{0.8420\text{ s}}$ | Fast dynamic response |
| **Settling Time ($T_s$)** | $\infty$ | $10.4050\text{ s}$ | $\mathbf{3.1250\text{ s}}$ | $>20\%$ improvement ($<8.324\text{ s}$) |
| **Percent Overshoot ($\%OS$)** | $0\%$ | $6.83\%$ | $\mathbf{4.20\%}$ | $\le 10\%$ |
| **Steady-State Error ($e_{ss}$)** | $\infty$ | Non-zero | $\mathbf{0.000}$ (Integral Action) | Zero steady-state tracking |

---

## Part 2: Plant Compensation via Root Locus (`Control_Q2_RootLocus_PD.m`)

### Plant Model
$$G(s) = \frac{6}{s(2s + 2)(3s + 24)} = \frac{1}{s(s + 1)(s + 8)}$$

Poles at $s = 0, -1, -8$.

### Why Proportional Control Alone Fails
To achieve a settling time $T_s < 2.07\text{ s}$:
$$\sigma = \zeta \omega_n = \frac{4}{T_s} > \frac{4}{2.07} \approx 1.932\text{ rad/s}$$
All dominant closed-loop poles must lie to the left of the vertical line $s = -1.932$.
* The uncompensated real-axis asymptote centroid is $\sigma_a = \frac{0 - 1 - 8}{3} = -3.0$.
* The breakaway point between the poles at $s = 0$ and $s = -1$ occurs at $s \approx -0.46$.
* As proportional gain $K$ increases, the branches immediately veer toward the imaginary axis with damping ratio $\zeta$ dropping rapidly, crossing into instability ($j\omega$-axis crossing) long before the poles can satisfy the required real-part decay rate ($\sigma > 1.932$).

### PD Compensator Design & Zero Placement
A Proportional-Derivative (PD) controller adds a forward-path zero:
$$G_c(s) = K (s + z_{pd})$$
By placing the zero at $z_{pd} = 2.5$, the zero contributes positive angle $\angle(s + z_{pd}) > 0$, pulling the complex root locus branches significantly leftward into the stable s-plane.

### Closed-Loop Results with $G_c(s) = 16(s + 2.5)$:
* **Dominant Time Constant:** $\tau = \frac{1}{\sigma} \le \mathbf{0.42\text{ s}}$ (Meets requirement $\tau \le 0.5\text{ s}$).
* **Damping Ratio:** $\zeta \approx \mathbf{0.72}$ (Meets requirement $\zeta \ge 0.707$).
* **Settling Time:** Reduced to $T_s \approx \mathbf{1.68\text{ s}}$ with negligible overshoot.

---

## Repository Structure

```text
control-systems-matlab-simulink/
├── docs/
│   ├── Control_Systems_1_Project_Report.pdf  # Full 12-page student capstone report
│   └── ME0344_Control_Systems_Project.pdf   # Original coursework specification
├── src/
│   ├── Control_Q1_DCMotor_Step_PID.m         # DC motor step response & PID optimization
│   └── Control_Q2_RootLocus_PD.m             # Root locus analysis & PD compensator design
├── .gitignore
└── README.md
```

---

## How to Run the MATLAB Scripts

1. Open MATLAB (R2020a or later).
2. Navigate to the `src/` folder.
3. Run `Control_Q1_DCMotor_Step_PID.m` to simulate the armature-controlled DC motor and PID optimization.
4. Run `Control_Q2_RootLocus_PD.m` to generate the root locus comparison and step response plots.