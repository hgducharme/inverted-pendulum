clc; clear; close all;

% Define system parameters
g = 9.807;          % m/s^2
massPendulum = 0.25; % kg
massCart = 0.320;    % kg
massSystem = massPendulum + massCart;
L = 0.15; % m
lengthTotal = L*2;
MOI = (1/3)*massPendulum*lengthTotal^2;
beta = massPendulum*L/(massPendulum*L^2 + MOI);
frictionCoeff = 0.5;
denominator = massCart + massPendulum*(1 - L*beta);

% This represents the value of cos(theta equilibrium). So if we want to
% analyze the up position, theta = 0, so cos(0) = 1. If we want to analyze
% the down position then theta = pi, cos(pi) = -1.
equilibriumPoint = 1;

% Create state space model
A = [0, 0, 1, 0;
     0, 0, 0, 1;
     g*beta*massSystem*equilibriumPoint/denominator, 0, 0, -frictionCoeff*beta*equilibriumPoint/denominator;
     massPendulum*g*L*beta/denominator, 0, 0, -frictionCoeff/denominator];
 
B = [0;
     0;
     beta*equilibriumPoint/denominator;
     1/denominator];
  
C = [1, 0, 0, 0;
     0, 1, 0, 0];
   
D = 0;

model = ss(A, B, C, D);
model.StateName = {'theta', 'x', 'thetaDot', 'xDot'};
model.OutputName = {'theta', 'x'};

%% Analyze stability and PID tuning

eigenvalues = eig(A);
transferFunctions = tf(model);
thetaTF = transferFunctions(1);
cartTF = transferFunctions(2);

s = tf('s');
integral = 1/s;
derivative = s;
controller = integral+derivative;

figure()
rlocus(thetaTF)

figure()
rlocus(integral*thetaTF)

zeros = [-2 -5.5];
poles = 0;
gain = 100;
controller = zpk(zeros, poles, gain);
figure()
rlocus(controller*thetaTF);
% p = [-0.8679 -4.2833 -2 0];
% K = place(A, B, p);
% syscl = ss(A-B*K, B, C,D);
% step(syscl);