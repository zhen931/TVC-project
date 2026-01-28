clc
clear
close all

%%%%% Rocket Simulator: Main

%% Params
ctrlmode = 'dummy';             % dummy, pid, rl, hybrid
thrustmap = 'linear';           % linear, curve
thrustdelay = 'ode1';           % ode1 (1st oder ODE), function handle
bodypolarmap = 'dummy';         % dummy, data
finpolarmap = 'dummy';          % dummy, data
envmap = 'dummy';               % dummy, stdatm

%% Config
% Loading directories
addpath(fullfile(pwd, 'Config/'), ...
        fullfile(pwd, 'Config/ThrustDelayFits/'), ...
        fullfile(pwd, 'Controllers/'), ...
        fullfile(pwd, 'Environment/'), ...
        fullfile(pwd, 'Simulation/'), ...
        fullfile(pwd, 'Simulation/TestModes/'), ...
        fullfile(pwd, 'Simulation/PathGen/'), ...
        fullfile(pwd, 'Simulation/Engines/'), ...
        fullfile(pwd, 'Simulation/Solvers/'), ...
        fullfile(pwd, 'Polars/'), ...
        fullfile(pwd, 'Polars/Body/'), ...
        fullfile(pwd, 'Polars/Fins/'));

% Loading rocket data
rocket = rocketInit(thrustmap, thrustdelay);

% Loading polar data
rocket.aero.bodymap = polarInit('body', bodypolarmap);
rocket.aero.finmap = polarInit('fin', finpolarmap);

% Loading environment model
[env.alt.gmap, env.alt.tmap, env.alt.rhomap, env.wind] = envInit(envmap);

%% Simulations
freedat = freeSim(rocket, ctrlmode);
MCdat = MCSim(rocket, ctrlmode);
pathdat = pathSim(rocket, ctrlmode);

%% Visualising and Analysing Data