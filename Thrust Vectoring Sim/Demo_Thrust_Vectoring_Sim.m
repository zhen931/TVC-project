clc
clear
close all

%%%%% Rocket Thrust Vectoring Simulation

%% Defining Problem
% Rocket Vector
rocket = [mass
          cg
          I
          Cl
          Cd
          cp
          T_poa
          T_max
          T_timeconst
          fin_delay
          S_ref
          S_fin];
sim = [position
       velocity
       omega
       ];


rocket = struct( ...
    'mass', 1, ...
    'cg', zeros(3,1), ...
    'I', zeros(3,3), ... %% Intertia tensor
    'Cl', 0.2, ...
    'Cd', 1.3, ...
    'cp', zeros(3,1), ...
    'T_poa', zeros(3,1), ... %% Thrust point of action
    'T_max', 100, ...
    'T_timeconst', 1.2, ...
    'fin_delay', 0.05, ...
    'S_ref', 0.1, ...
    'S_fin', 0.01);
sim = struct( ...
    'dt', 1e-5, ...
    'Integrator', 'ExplicitEuler', ... %% lol
    'call_steps', 1e3);
state = struct( ...
    'position', zeros(3,1), ...
    'velocity', zeros(3,1), ...
    'omega', zeros(3,1), ...
    'q', [1; 0; 0; 0], ... %% Quaternion
    'T', 0);
control = struct( ...
    'throttle', 0, ...
    'gimbal_pitch', 0, ...
    'gimbal_yaw', 0, ...
    'fin', 0);
env = struct( ...
    'g', -9.81, ...
    'temp', 288, ...
    'rho', 1.225, ...
    'wind', zeros(3,1));

%% Test parameters
% Rocket test parameters
rocket = struct( ...
    'mass', 5, ...                 % kg
    'cg', [0; 0; 0.3], ...            % m, center of gravity
    'I', [2,0.1,0; 0.1,1.5,0; 0,0,1], ... % kg·m², 3x3 inertia tensor
    'Cl', 0.8, ...                  % lift coefficient
    'Cd', 0.5, ...                  % drag coefficient
    'cp', [0; 0; 0.25], ...          % m, center of pressure
    'T_poa', [0; 0; 0.3], ...         % m, thrust point of action
    'T_max', 200, ...               % N, max thrust
    'T_timeconst', 0.2, ...         % s, thrust lag time constant
    'fin_delay', 0.05, ...          % s, fin actuation lag
    'S_ref', 0.01, ...              % m², reference area
    'S_fin', 0.005);                 % m², fin area

% Simulation parameters
sim = struct( ...
    'dt', 1e-5, ...                 % s, integration timestep
    'Integrator', 'ExplicitEuler', ... 
    'call_steps', 10e5);             % number of integration steps per call

% Initial rocket state
state = struct( ...
    'position', [0; 0; 0], ...      % m
    'velocity', [0; 0; 0], ...      % m/s
    'omega', [0; 0; 0], ...         % rad/s
    'q', [1; 0; 0; 0], ...          % unit quaternion
    'T', 0);                        % current thrust magnitude (N)

% Control inputs
control = struct( ...
    'throttle', 0.7, ...            % 0-1
    'gimbal_pitch', deg2rad(0), ... % rad
    'gimbal_yaw', deg2rad(0), ...  % rad
    'fin', 0);                      

% Environment
env = struct( ...
    'g', -9.81, ...                  % m/s²
    'temp', 288, ...                % K
    'rho', 1.225, ...               % kg/m³
    'wind', [1; 0; 0]);             % m/s, wind along x-axis



%% Gimbal Angle Controller 



%% Visualisation and Data Acquisition
% Rocket plot
figure('Name','Rocket Simulation','Color','w')
hold on
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
% xlim([-50 50])
% ylim([-50 50])
% zlim([-50 50])
axis equal
grid on
view(3)

% Drawing the ground
[Xg, Yg] = meshgrid(-50:10:50, -50:10:50);
Zg = zeros(size(Xg));
surf(Xg, Yg, Zg, 'FaceColor','g', 'EdgeColor','none');

% Drawing rocket properties
rocket_marker = plot3(0,0,0, 'ro', 'MarkerSize',6, 'MarkerFaceColor', 'r');
trajectory = animatedline('Color','b', 'LineWidth',2);
thrust_vector = quiver3(0,0,0,0,0,0, 'r', 'LineWidth',2, 'MaxHeadSize',2);
wind_vector = quiver3(0,0,0,0,0,0, 'g', 'LineWidth',2, 'MaxHeadSize',2);


% Diagnostic Plots
figure('Name','Motion in Z','Color','w')
hold on
xlabel('t [s]')
ylabel('Thing [whatever]')
grid on
Z_force = animatedline('Color','r', 'LineWidth',2);
Z_a = animatedline('Color','b', 'LineWidth',2);
hold off

figure('Name','Force Contributions','Color','w')
hold on
xlabel('t [s]')
ylabel('F [N]')
grid on
Aero_contribution = animatedline('Color','b', 'LineWidth',2);
Thrust_contribution = animatedline('Color','r', 'LineWidth',2);
Weight_contribution = animatedline('Color','g', 'LineWidth',2);
legend("Aero", "Thrust", "Weight")


%% Physics Engine
% Defining performance metrics
path = zeros(3,sim.call_steps);
velo_hist = zeros(3,sim.call_steps);
a_hist = zeros(3,sim.call_steps);

% Iterating
for i = 1:sim.call_steps

% Extracting Quaternion and dt for speed
q = state.q;
dt = sim.dt;

% Thrust
% Simple linearised model
state.T =  state.T + dt/rocket.T_timeconst * (rocket.T_max*control.throttle - state.T);

% Gimballing thrust force
g_pitch = control.gimbal_pitch;
g_yaw = control.gimbal_yaw;
T_b = state.T * [sin(g_yaw)*cos(g_pitch); -sin(g_pitch); cos(g_pitch)*cos(g_yaw)];
T_w = BodytoWorld(q, T_b);

% Aerodynamic Forces
% Getting body velocity
v_b = WorldtoBody(q, state.velocity - env.wind);
V = sqrt(v_b(1)^2 + v_b(2)^2 + v_b(3)^2);

% Angles of incidence and sideslip and dynamic pressure
alpha = atan2(v_b(2), v_b(3));
beta = atan2(v_b(1), v_b(3));
q_dyn = 0.5 * env.rho * V^2;

% Limiting forces else the solver explodes
alpha = min(alpha, pi/20);
beta = min(beta, pi/20);

% Rocket aero forces
% Drag
D_mag = q_dyn * rocket.Cd * rocket.S_ref;
D = -D_mag * v_b / max(V, 1e-5); %% 1e-5 used to prevent NaN at V = 0

% Lift
Cl_a = 2*pi; %% assumed constant lift slope
L_mag = 3 * q_dyn * rocket.Cl * rocket.S_fin * Cl_a * alpha; %% update this to have a lookup table instead of multiplying by constant slope

% Sideslip 
Cy_b = 2*pi; %% assumed constant lift slope
Y_mag = q_dyn * rocket.S_fin * Cy_b * beta;
disp("q_dyn")
L = [Y_mag; L_mag; 0];


% Total force
F_aero_b = D + L;
F_aero_w = BodytoWorld(q, F_aero_b);
F_w = T_w + F_aero_w + [0; 0; rocket.mass*env.g];
disp("Aero force: " + F_w)
disp("")

% Moments
l_T = rocket.T_poa - rocket.cg;
M_T = [
    l_T(2)*T_b(3) - l_T(3)*T_b(2);
    l_T(3)*T_b(1) - l_T(1)*T_b(3);
    l_T(1)*T_b(2) - l_T(2)*T_b(1)]; 

l_aero = rocket.cp - rocket.cg;
M_aero = [
    l_aero(2)*F_aero_b(3) - l_aero(3)*F_aero_b(2);
    l_aero(3)*F_aero_b(1) - l_aero(1)*F_aero_b(3);
    l_aero(1)*F_aero_b(2) - l_aero(2)*F_aero_b(1)]; 

M_b = M_T + M_aero;


% Angular Dynamics
w1 = state.omega(1);
w2 = state.omega(2);
w3 = state.omega(3);

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
state.q = q + dt * q_dot;
state.q = state.q / (sqrt(state.q(1)^2 + state.q(2)^2 + state.q(3)^2 + state.q(4)^2));


% Mass depletion: TBA


% Updating state
state.omega = state.omega + dt * omega_dot;
a = F_w / rocket.mass;
state.velocity = state.velocity + dt * a;
state.position = state.position + dt * state.velocity;


% Updating performance metrics
path(:,i) = state.position;
velo_hist(:,i) = state.velocity;
a_hist(:,i) = a;

% Debug
disp(state.velocity)

% Updating the plot every 1000 iterations
if mod(i, 100) == 0
    t = dt*i;
    
    addpoints(trajectory, state.position(1), state.position(2), state.position(3));
    addpoints(Z_force, t, F_w(3));
    addpoints(Aero_contribution, t, norm(F_aero_w));
    addpoints(Thrust_contribution, t, norm(T_w));
    addpoints(Weight_contribution, t, norm([0; 0; rocket.mass*env.g]));
    
    % Update rocket marker position
    set(rocket_marker, 'XData', state.position(1), ...
                      'YData', state.position(2), ...
                      'ZData', state.position(3));

    % Update thrust arrow (in world frame)
    set(thrust_vector, 'XData', state.position(1), ...
                     'YData', state.position(2), ...
                     'ZData', state.position(3), ...
                     'UData', T_w(1), ...
                     'VData', T_w(2), ...
                     'WData', T_w(3));

    % Update wind arrow to follow rocket
    set(wind_vector, 'XData', state.position(1), ...
                   'YData', state.position(2), ...
                   'ZData', state.position(3), ...
                   'UData', 5*env.wind(1), ...
                   'VData', 5*env.wind(2), ...
                   'WData', 5*env.wind(3));

    drawnow limitrate
end
end

%% State Update and Sensors










%% Digital Twinning





%% Functions

function [x] = BodytoWorld(q, x)
%%%%% Transforms Body vector to World
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
x1 = x(1);
x2 = x(2); 
x3 = x(3);

x = [
    (q1^2+q2^2-q3^2-q4^2)*x1 + 2*(q2*q3-q1*q4)*x2 + 2*(q2*q4+q1*q3)*x3;
    2*(q2*q3+q1*q4)*x1 + (q1^2-q2^2+q3^2-q4^2)*x2 + 2*(q3*q4-q1*q2)*x3;
    2*(q2*q4-q1*q3)*x1 + 2*(q3*q4+q1*q2)*x2 + (q1^2-q2^2-q3^2+q4^2)*x3
];
end


function [x] = WorldtoBody(q, x)
%%%%%% Transforms World vector to Body
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
x1 = x(1);
x2 = x(2); 
x3 = x(3);

x = [
    (q1^2+q2^2-q3^2-q4^2)*x1 + 2*(q2*q3+q1*q4)*x2 + 2*(q2*q4-q1*q3)*x3;
    2*(q2*q3-q1*q4)*x1 + (q1^2-q2^2+q3^2-q4^2)*x2 + 2*(q3*q4+q1*q2)*x3;
    2*(q2*q4+q1*q3)*x1 + 2*(q3*q4-q1*q2)*x2 + (q1^2-q2^2-q3^2+q4^2)*x3
];
end