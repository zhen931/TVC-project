clc
clear
close all

%%%%% Rocket Simulator: Main

%% Params
ctrlmode = 'dummy';             % dummy, pid, rl, hybrid
thrustmode = 'linear';          % linear, curve
thrustfit = 'autofit';      % autofit, function handle

%% Config
% Loading directories
addpath(fullfile(pwd, 'Config/'), ...
        fullfile(pwd, 'Config/ThrustFits/'), ...
        fullfile(pwd, 'Controllers/'), ...
        fullfile(pwd, 'Sims/'));

% Loading rocket data
rocket = rocketInit(thrustmode, thrustfit);

% Testing thrust fit function
testThrust = rocket.T.fit(rocket);
disp(testThrust)

%% Simulations
freedat = freeSim(rocket, ctrlmode);
MCdat = MCSim(rocket, ctrlmode);
pathdat = pathSim(rocket, ctrlmode);

%% Visualising and Analysing Data