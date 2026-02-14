function [state_dot] = Physics(rocket, x, control, env)
%%%%% Physics engine that determines state derivatives
%%% Inputs:
%%% Rocket struct
%%% State vector 
%%% Control vector
%%% Environment struct

% state vector:
% position, velocity, q, omega, T, m, cg, cp

% control vector:
% throttle, gimbal angles, fin angles

% env vector:
% g, rho, T, windspeed



%% Logging Rocket Physical Parameteres to improve compute
% rocket = struct( ...
%     'mass', 1, ...
%     'cg', zeros(3,1), ...
%     'I', zeros(3,3), ... %% Intertia tensor
%     'Cl', 0.2, ...
%     'Cd', 1.3, ...
%     'cp', zeros(3,1), ...
%     'T_poa', zeros(3,1), ... %% Thrust point of action
%     'T_max', 100, ...
%     'T_timeconst', 1.2, ...
%     'fin_delay', 0.05, ...
%     'S_ref', 0.1, ...
%     'S_fin', 0.01);
% sim = struct( ...
%     'dt', 1e-5, ...
%     'Integrator', 'ExplicitEuler', ... %% lol
%     'call_steps', 1e3);
% state = struct( ...
%     'position', zeros(3,1), ...
%     'velocity', zeros(3,1), ...
%     'omega', zeros(3,1), ...
%     'q', [1; 0; 0; 0], ... %% Quaternion
%     'T', 0);
% control = struct( ...
%     'throttle', 0, ...
%     'gimbal_pitch', 0, ...
%     'gimbal_yaw', 0, ...
%     'fin', 0);
% env = struct( ...
%     'g', -9.81, ...
%     'temp', 288, ...
%     'rho', 1.225, ...
%     'wind', zeros(3,1));

%% Preparing Data

% Unpacking x
r = x(1:3);
v = x(4:6);
q = x(7:10);
omega = x(11:13);
T = x(14);
m = x(15);
cg = x(16:18);
cp = x(19:21);

% Unpacking control struct
g_pitch = control.gimbal_pitch;
g_yaw = control.gimbal_yaw;
gimbal = [0, 0, 0; 0, 0, 0; 0, 0, 0];


%% Thrust
dT = rocket.T.delay(T, rocket.T.map(control.throttle)) * rocket.T.max;
T_b = [T,0,0] * []; % write out matrix operation manually for compute
T_w = BodytoWorld(q, T_b);


%% Mass
dm = rocket.mass.burn(m);


%% Aero

% Getting body velocity and dynamic pressure
v_b = WorldtoBody(q, v - env.wind);
V_b = sqrt(v_b(1)^2 + v_b(2)^2 + v_b(3)^2);
v_hat = v_b / V_b;
q_dyn = 0.5 * env.rho * V_b^2;

% Sideslip (rad)
x_b = [1;0;0];
v_n = v_hat - dot(v_hat, x_b) * x_b; % implement manually
V_n = sqrt(v_n(1)^2 + v_n(2)^2 + v_n(3)^2);
cos_alpha = dot(v_hat, x_b); % ensure between [-1,1]; implement manually


%% Total Force and Moments

% Forces
F_aero_w = BodytoWorld(q, F_aero_b);
F_w = T_w + F_aero_w + [0; 0; m*env.g];

% Moments


%% Rotational Dynamics

omegaI = [
    w1*rocket.I(1,1) + w2*rocket.I(1,2) + w3*rocket.I(1,3);
    w1*rocket.I(2,1) + w2*rocket.I(2,2) + w3*rocket.I(2,3);
    w1*rocket.I(3,1) + w2*rocket.I(3,2) + w3*rocket.I(3,3)];

omega_cross_omegaI = [
    w2*omegaI(3) - w3*omegaI(2);
    w3*omegaI(1) - w1*omegaI(3);
    w1*omegaI(2) - w2*omegaI(1)];

omega_dot = rocket.I \ (M_b - omega_cross_omegaI);


% Quaternion Integration
q_dot = 0.5 * [
    -q(2)*w1 - q(3)*w2 - q(4)*w3;
    q(1)*w1 + q(3)*w3 - q(4)*w2;
    q(1)*w2 - q(2)*w3 + q(4)*w1;
    q(1)*w3 + q(2)*w2 - q(3)*w1];


%% State Derivative Update

state_dot(1:3) = v;
state_dot(4:6) = [];
state_dot(14) = dT;


end