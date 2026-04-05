function [state_dot, debug] = Physics(t, rocket, state, control, env, write)
%%%%% Physics engine that determines state derivatives
%%% Inputs:
%%% Time
%%% Rocket struct
%%% State vector 
%%% Control vector
%%% Environment struct
%%% Write boolean

% state vector:
% position, velocity, q, omega, T, m, cg, cp

% control vector:
% throttle, gimbal angles, fin angles

% env vector:
% g, rho, Temp, windspeed



%% Preparing Data
% Unpacking state
r = state(1:3);
v = state(4:6);
q = state(7:10); 
omega = state(11:13);
T = state(14);               % Thrust, for delay modelling
m_fuel_norm = state(15);     % Fraction between 0-1
g_yaw = state(16);           % Gimbal angles, for delay modelling
g_pitch = state(17);

% Unpacking control
throttle = control.throttle;
g_yaw_des = control.g_yaw;
g_pitch_des = control.g_pitch;



%% State Preconditioning
% Renormalise quaternion
q = q / max(norm(q), 1e-6);



%% Control Preconditioning
% Rotation Matrices
R_yaw = [cos(g_yaw),  0, sin(g_yaw);
             0,            1, 0;
            -sin(g_yaw),  0, cos(g_yaw)];
R_pitch = [cos(g_pitch), -sin(g_pitch), 0;
               sin(g_pitch),  cos(g_pitch), 0;
               0,             0,            1];
gimbal_des = R_yaw * R_pitch;



%% Environment
alt = r(3);
g = env.alt.gmap(alt);
rho = env.alt.rhomap(alt);
temp = env.alt.tmap(alt);
p = env.alt.pmap(alt);
wind = env.windmap(alt, t);



%% Inertial Properties
% CG and Inertia Tensor
total_mass = rocket.mass.m_dry + m_fuel_norm*rocket.mass.m_maxfuel;
CG = rocket.mass.map(m_fuel_norm);
I = rocket.inertia.map(m_fuel_norm);



%% Gimbal Delay
dg_yaw = rocket.gimbal.delay(g_yaw, g_yaw_des);
dg_pitch = rocket.gimbal.delay(g_pitch, g_pitch_des);



%% Thrust
% Enforcing no negative mass
fuel_remaining = double(m_fuel_norm > 0);
m_fuel_norm = max(m_fuel_norm, 0);

% Thrust delay
dT = rocket.T.delay(T, rocket.T.map(throttle) * rocket.T.max);
dT = dT * fuel_remaining;

% Pressure-corrected Thrust (can be -ve from pressure)
T_eff = T - (rocket.T.ptest - p) * rocket.geom.A_e;
T_eff = max(T_eff, 0);
T_eff = T_eff * fuel_remaining;
T = T * fuel_remaining;

% Rotating
T_b = gimbal_des * [T_eff;0;0];



%% Specific Impulse and Fuel Burn
spI_sl = rocket.T.spImpulsemap(throttle);
dm_fuel_norm = -T / (spI_sl * 9.81 * rocket.mass.m_maxfuel); % g0 is intentional

% Prevent negative fuel
dm_fuel_norm = dm_fuel_norm * fuel_remaining;




%% Aero - Preprocessing
% Body frame velocity
x_b = [1; 0; 0];
v_b = World2Body(q, v - wind);
u_ax = v_b(1);
v_lat = v_b(2);   % yaw plane
w_lat = v_b(3);   % pitch plane

V_b = max(sqrt(u_ax^2 + v_lat^2 + w_lat^2), 1e-6); % tolerance to stop inf at rest

% Checking if there is any body velcoity 
if V_b < 1e-6
    v_hat       = x_b;
    alpha       = 0;
    beta        = 0;
    alpha_total = 0;
    V_b         = 1e-6;
else
    v_hat       = v_b / V_b;
    u_sign      = sign(u_ax) + double(sign(u_ax) == 0);
    
    % Calculating signed Alpha (pitch) and Beta (sideslip)
    alpha       = atan2(w_lat, max(abs(u_ax), 1e-6)) * u_sign;
    beta        = atan2(v_lat, max(abs(u_ax), 1e-6)) * u_sign;
    alpha_total = acos(max(min(dot(v_hat, x_b), 1), -1));
end

% Normal force unit vectors in body frame
n_pitch = [0; 0; 1];
n_yaw = [0; 1; 0];

% % Sideslip angle and unit vector
% v_n = v_hat - dot(v_hat, x_b) * x_b;
% V_n = sqrt(v_n(1)^2 + v_n(2)^2 + v_n(3)^2);
% cos_alpha = dot(v_hat, x_b); % ensure between [-1,1]; implement manually


% Mach, Re
a_sound = sqrt(env.const.gamma * env.const.R * temp);
M = V_b / a_sound;
mu = 1.716e-5 * (temp/273.15)^1.5 * (273.15+110.4)/(temp+110.4); % Sutherland
Re = rho * V_b * rocket.geom.L_ref / mu;

% Scaling Factors
q_dyn = 0.5 * rho * V_b^2;
coeff2force = q_dyn * rocket.geom.S_ref;



%% Aero Forces
Ct_cone = rocket.aero.cone.Ctmap(M, alpha_total);
Cn_cone = rocket.aero.cone.Cnmap(M, alpha_total);

Ct_body = rocket.aero.body.Ctmap(M, alpha_total);
Cn_body = rocket.aero.body.Cnmap(M, alpha_total);

% Single fin, apply multi-fin correction
Ct_fin  = rocket.aero.fin.Ctmap(M, alpha_total)  * rocket.geom.n_fins;
Cn_fin  = rocket.aero.fin.Cnmap(M, alpha_total)  * rocket.geom.n_fins;

Ct_total = Ct_cone + Ct_body + Ct_fin;
Cn_total = Cn_cone + Cn_body + Cn_fin;

% Project Cn onto pitch and yaw axes
Cn_yaw   = Cn_total * sin(beta);
Cn_pitch = Cn_total * sin(alpha);

% Project onto body frame
F_aero_b = -Ct_total * coeff2force * x_b + ...
            Cn_pitch  * coeff2force * n_pitch + ...
            Cn_yaw    * coeff2force * n_yaw;



%% Aero Moments
% Getting CP
CP_cone = rocket.aero.cone.CPmap(M, alpha_total);
CP_body = rocket.aero.body.CPmap(M, alpha_total);
CP_fin = rocket.aero.fin.CPmap(M, alpha_total);

% Overall CP (THIS IS GOING TO BE VERY BUGGY)
Cn_denom = Cn_total + 1e-6;             % avoid div by zero at alpha=0
CP_total = (Cn_cone * CP_cone + ...
            Cn_body * CP_body + ...
            Cn_fin  * CP_fin) / Cn_denom;

% Aero Moment
aero_arm  = CP_total - CG(1);
M_pitch_aero = -aero_arm * Cn_pitch * coeff2force;   % about y_b, nose-up positive
M_yaw_aero   =  aero_arm * Cn_yaw   * coeff2force;   % about z_b
M_roll_aero  = 0;     
M_aero_b = [M_roll_aero; M_yaw_aero; M_pitch_aero];

% Thrust Moment
thrust_arm = rocket.geom.x_engine - CG(1);
M_thrust_b = thrust_arm * cross(x_b, T_b);



%% Total Force and Moments
% Forces
F_b = F_aero_b + T_b;
F_g = [0; 0; -double(alt~=0) * total_mass*g];
F_w = Body2World(q, F_b) + F_g;
a_w = F_w / total_mass;

% Moments
M_b = M_aero_b + M_thrust_b;



%% Rotational Dynamics
omegaI = [
    omega(1)*I(1,1) + omega(2)*I(1,2) + omega(3)*I(1,3);
    omega(1)*I(2,1) + omega(2)*I(2,2) + omega(3)*I(2,3);
    omega(1)*I(3,1) + omega(2)*I(3,2) + omega(3)*I(3,3)];

omega_cross_omegaI = [
    omega(2)*omegaI(3) - omega(3)*omegaI(2);
    omega(3)*omegaI(1) - omega(1)*omegaI(3);
    omega(1)*omegaI(2) - omega(2)*omegaI(1)];

omega_dot = I \ (M_b - omega_cross_omegaI);
omega_dot(~isfinite(omega_dot)) = 0;

% Quaternion Derivative
q_dot = 0.5 * [
    -q(2)*omega(1) - q(3)*omega(2) - q(4)*omega(3);
    q(1)*omega(1) + q(3)*omega(3) - q(4)*omega(2);
    q(1)*omega(2) - q(2)*omega(3) + q(4)*omega(1);
    q(1)*omega(3) + q(2)*omega(2) - q(3)*omega(1)];



%% State Derivative Update
state_dot = zeros(17,1);
state_dot(1:3) = v;
state_dot(4:6) = a_w;
state_dot(7:10) = q_dot;
state_dot(11:13) = omega_dot;
state_dot(14) = dT;
state_dot(15) = dm_fuel_norm;
state_dot(16) = dg_yaw;
state_dot(17) = dg_pitch;



%% Data History
if nargout > 1 && write
    debug.t              = t;
    debug.alpha          = alpha;
    debug.M              = M;
    debug.Re             = Re;
    debug.q_dyn          = q_dyn;
    debug.T_eff          = T_eff;
    debug.mdot_norm      = dm_fuel_norm;
    debug.CP_total       = CP_total;
    debug.CG             = CG;
    debug.aero_arm       = aero_arm;
    debug.Ct_total       = Ct_total;
    debug.Cn_total       = Cn_total;
    debug.F_aero_b       = F_aero_b;
    debug.T_b            = T_b;
    debug.M_b            = M_b;
    debug.a_w            = a_w;
    debug.omega_dot      = omega_dot;
    debug.fuel_remaining = fuel_remaining;
else
    debug = [];
end
end