clc;
clear;
close all;

s = tf('s');

G = [0.05/(s^2+0.32*s+1.71),  0.095/(s^2+0.32*s+1.71);
    -0.12/(s^2+s),            0.1/(s^2+s)];


%% Plant

A = [ 0     1      0     0;
     -1.71 -0.32   0     0;
      0     0      0     1;
      0     0      0    -1];

B = [ 0      0;
      0.05   0.095;
      0      0;
     -0.12   0.1];

C = [1 0 0 0;
     0 0 1 0];

D = zeros(2,2); 

%Augmented plant 

Aa = [A zeros(4,2);
     -C zeros(2,2)];

Ba = [B;
      zeros(2,2)];

Ea = [zeros(4,2);
      eye(2)];

%% Canonical input matrix

B0 = [0 0;
      1 0;
      0 0;
      0 1];

Gamma = [ 0.05  0.095;
         -0.12  0.10];

B0a = [B0;
       zeros(2,2)];

%% Reference model
% T(s)= 4/(s^2+2s+4)

a0m = 4;
a1m = 2;

a2m = 4;
a3m = 2;

Am = [0     1     0     0;
     -a0m -a1m   0     0;
      0     0     0     1;
      0     0   -a2m -a3m];

Bm = [0    0;
      a0m 0;
      0    0;
      0   a2m];

Cm = [1 0 0 0;
      0 0 1 0];

%Augmented Reference

Aam = [0 1 0 0 0 0;
      -8 -4 0 0 8 0;
       0 0 0 1 0 0;
       0 0 -8 -4 0 8;
       -1 0 0 0 0 0;
       0 0 -1 0 0 0];

Bam = [zeros(4,2);
       eye(2)];



% K_star = pinv(B)*(A-Am);
% L_star = inv(B'*B)*B'*Bm;

%% Lyapunov matrix


% for pd %
 % GammaK = 500*eye(2);
 % Q = eye(4);
 % P = lyap(Am',Q);
 % S = GammaK*B0'*P;

 % for pid %
GammaK = 50000*eye(2);
Q = eye(6);
P = lyap(Aam',Q);
S = GammaK*B0a'*P;

%
wn= 1000;
zeta= 0.5;
lamda = 2;
gama = 500;
alpha = 10;



%%



K_star = pinv(B)*(A-Am);

L_star = pinv(B)*Bm;

%Kdot = -GammaK*Gamma'*(B0'*P*e)*x';

%Bns=[B(2,:); B(4,:)];

%T=([0 0 1 0;1 0 0 0;0 0 0 1;0 1 0 0]*[inv(Bns) zeros(2,2);zeros(2,2) eye(2)]*[0 1 0 0;0 0 0 1;1 0 0 0;0 0 1 0]); 
%Bt=inv(P)*B;
%At=inv(P)*A*P;