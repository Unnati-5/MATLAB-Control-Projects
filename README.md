\# MATLAB \& Simulink Control Projects



\## Overview



This repository contains MATLAB and Simulink implementations developed for

control-system modeling, simulation, and controller design.



The projects focus on modeling of dynamic systems, reference-model design,

adaptive control, and simulation-based validation.



\---



\## Projects



\### 1. TRMS Control and MRAC



Implementation and simulation of control strategies for a

Twin Rotor MIMO System (TRMS).



The work includes:



\- TRMS mathematical modeling

\- State-space representation

\- Transfer-function modeling

\- Reference-model design

\- Augmented plant modeling

\- Integral-state augmentation

\- Indirect Model Reference Adaptive Control (MRAC)

\- Lyapunov-based adaptive control formulation

\- Parameter estimation

\- MATLAB/Simulink simulation



\### Controller Configurations



Two controller configurations are considered:



\- PD-based control

\- PID-based control with integral-state augmentation



The PID formulation uses an augmented six-state model to incorporate

integral action for improved tracking behavior.



\---



\## TRMS Model



The TRMS is modeled as a coupled multi-input multi-output system.



The state-space representation is used for controller design and simulation.



The project includes:



\- Plant matrices `A`, `B`, `C`, and `D`

\- Augmented plant model

\- Reference model

\- Augmented reference model

\- Canonical input matrix

\- Lyapunov matrix calculation

\- Ideal controller gain calculation



\---



\## MRAC Design



The indirect MRAC formulation estimates plant parameters and uses the

estimated model to determine controller parameters.



The Lyapunov-based design uses:



```text

A'P + PA = -Q

