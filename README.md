# Control Systems I: MATLAB, Simulink, Root Locus, and PID Design

**Course:** ME0344 - Control Systems I
**Institution:** German Jordanian University (GJU)

## Overview

This repository contains selected MATLAB/Simulink coursework for classical feedback-control design. The work covers DC-motor modeling, open- and closed-loop response analysis, root-locus design, and controller tuning against transient-response requirements.

## Included Work

### DC Motor Analysis and PID Control

Modeled a DC motor transfer function and compared open-loop and closed-loop step responses in MATLAB/Simulink. A PID controller was tuned to remove steady-state error and reduce settling time by approximately 20 percent.

### Root Locus and PD Controller Design

For the plant:

```math
G_p(s) = \frac{6}{s(2s+2)(3s+24)}
```

the project evaluates proportional-control feasibility and designs a PD controller using root locus. The design targets a dominant time constant of 0.5 s or less and a damping ratio of at least 0.707.

`src/Control_Q2.m` implements the selected PD controller, forms the unity-feedback system, displays the root locus and design grid, and reports step-response metrics.

## Repository Structure

```text
├── src/
│   └── Control_Q2.m
├── docs/
│   └── ME0344_Control_Systems_Project.pdf
└── README.md
```

## Run Locally

GitHub does not render MATLAB `.m` results or Simulink models interactively. Download the repository, open `src/Control_Q2.m` in MATLAB with Control System Toolbox, and run the script to reproduce the root-locus and step-response analysis.
