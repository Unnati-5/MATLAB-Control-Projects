%% TRMS - Model and MRAC Controller Setup
% =========================================================================
% Description:
%   State-space modeling and reference-model formulation for a
%   Twin Rotor MIMO System (TRMS).
%
%   This script prepares:
%       1. TRMS plant transfer-function and state-space models
%       2. Augmented plant model with integral states
%       3. Canonical input matrix
%       4. Reference model
%       5. Augmented reference model
%       6. Lyapunov matrix for adaptive controller design
%       7. Ideal controller parameters for comparison
%
% Control Method:
%   Indirect Model Reference Adaptive Control (MRAC)
%
% MATLAB / Simulink
% =========================================================================

clc;
clearvars;
close all;


%% 1. TRMS Transfer-Function Model
% -------------------------------------------------------------------------

s = tf('s');

G = [ ...
     0.05 / (s^2 + 0.32*s + 1.71),   0.095 / (s^2 + 0.32*s + 1.71);
    -0.12 / (s^2 + s),                0.10  / (s^2 + s)
    ];


%% 2. TRMS State-Space Model
% -------------------------------------------------------------------------
% State vector:
%
%   x = [x1  x2  x3  x4]'
%
% where the states represent the dynamics of the two coupled TRMS axes.

A = [ ...
     0       1       0       0;
    -1.71   -0.32     0       0;
     0       0       0       1;
     0       0       0      -1
    ];

B = [ ...
     0       0;
     0.05    0.095;
     0       0;
    -0.12    0.10
    ];

C = [ ...
     1   0   0   0;
     0   0   1   0
    ];

D = zeros(2,2);


%% 3. Augmented Plant Model
% -------------------------------------------------------------------------
% Integral states are introduced to eliminate steady-state tracking error.
%
% Augmented state vector:
%
%   xa = [x1 x2 x3 x4 z1 z2]'

Aa = [ ...
     A,              zeros(4,2);
    -C,              zeros(2,2)
    ];

Ba = [ ...
     B;
     zeros(2,2)
    ];

Ea = [ ...
     zeros(4,2);
     eye(2)
    ];


%% 4. Canonical Input Matrix
% -------------------------------------------------------------------------
% B0 represents the desired canonical input structure used in the
% adaptive-controller formulation.

B0 = [ ...
     0   0;
     1   0;
     0   0;
     0   1
    ];

B0a = [ ...
     B0;
     zeros(2,2)
    ];

% Input coupling matrix
Gamma = [ ...
     0.05    0.095;
    -0.12    0.10
    ];


%% 5. Reference Model
% -------------------------------------------------------------------------
% Desired second-order reference dynamics:
%
%       4
%   ---------
%   s^2 + 2s + 4
%
% Standard form:
%
%   s^2 + a1m*s + a0m

a0m = 4;
a1m = 2;

a2m = 4;
a3m = 2;

Am = [ ...
     0       1       0       0;
    -a0m   -a1m      0       0;
     0       0       0       1;
     0       0     -a2m    -a3m
    ];

Bm = [ ...
     0      0;
     a0m    0;
     0      0;
     0      a2m
    ];

Cm = [ ...
     1   0   0   0;
     0   0   1   0
    ];


%% 6. Augmented Reference Model
% -------------------------------------------------------------------------
% Augmented reference model including integral states.
%
% Augmented state vector:
%
%   xa_m = [x1m x2m x3m x4m z1m z2m]'

Aam = [ ...
     0    1    0    0    0    0;
    -8   -4    0    0    8    0;
     0    0    0    1    0    0;
     0    0   -8   -4    0    8;
    -1    0    0    0    0    0;
     0    0   -1    0    0    0
    ];

Bam = [ ...
     zeros(4,2);
     eye(2)
    ];


%% 7. Lyapunov-Based Adaptive Controller Design
% -------------------------------------------------------------------------
% Two controller configurations are considered:
%
%   1. PD-MRAC:
%      Uses the original 4-state plant/reference model.
%
%   2. PID-MRAC:
%      Uses the augmented 6-state model with integral states.
%
% The Lyapunov equation is:
%
%       A'P + PA = -Q
%
% where Q > 0 and P > 0.
%
% The adaptive update matrix is calculated as:
%
%       S = GammaK * B0' * P
%
% for the PD configuration, and
%
%       S = GammaK * B0a' * P
%
% for the PID configuration.
% -------------------------------------------------------------------------


%% 7.1 PD-MRAC Configuration
% -------------------------------------------------------------------------
% PD configuration uses the 4-state reference model Am and canonical
% input matrix B0.

GammaK_PD = 500 * eye(2);

Q_PD = eye(4);

P_PD = lyap(Am', Q_PD);

S_PD = GammaK_PD * B0' * P_PD;


%% 7.2 PID-MRAC Configuration
% -------------------------------------------------------------------------
% PID configuration uses the augmented 6-state reference model Aam.
% The additional two states represent integral action.

GammaK_PID = 50000 * eye(2);

Q_PID = eye(6);

P_PID = lyap(Aam', Q_PID);

S_PID = GammaK_PID * B0a' * P_PID;

%% 8. Adaptation and Controller Tuning Parameters
% -------------------------------------------------------------------------

wn    = 1000;     % Auxiliary bandwidth parameter
zeta  = 0.5;      % Damping ratio
lambda = 2;       % Adaptation/filter parameter
gamma  = 500;     % Adaptation gain
alpha  = 10;      % Normalization / auxiliary tuning parameter


%% 9. Ideal Controller Parameters
% -------------------------------------------------------------------------
% These parameters represent the ideal controller gains obtained when
% the exact plant model is known.
%
% K_star provides state-feedback gains.
% L_star provides reference-input/feedforward gains.

K_star = pinv(B) * (A - Am);

L_star = pinv(B) * Bm;


%% 10. Display Important Results
% -------------------------------------------------------------------------

disp('======================================================');
disp('          TRMS MRAC MODEL SETUP');
disp('======================================================');

disp('State-space matrix A:');
disp(A);

disp('Input matrix B:');
disp(B);

disp('Reference model Am:');
disp(Am);

disp('Ideal state-feedback gain K_star:');
disp(K_star);

disp('Ideal feedforward gain L_star:');
disp(L_star);

disp('Lyapunov matrix P:');
disp(P);

disp('======================================================');
disp('Model initialization completed.');
disp('======================================================');