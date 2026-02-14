% housekeeping
clc; clear; close all;

% Rocket & PID initialisation
% SI units (no Americans harmed)

leverArm = 0.3; % dstance CoM to gimbal pivot 

% PID gain values (NEEDS TUNING)
Kp_att = 6.0;
Kp_rate = 0.2;
Kd_rate = 0.05;
Ki_rate = 0.1;

