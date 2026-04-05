clc
clear
close all

%%%%% Rocket Simulator

%% Params                             % [available options]
opts.rocket.dat = 'rocketDat.xlsx';   % [filename]

opts.mass.map = 'linear';             % linear, [filename]
opts.inertia.map = 'linear';          % linear, [filename]

opts.spimpulse.ptest = 101325;        % test pressure [Pa]
opts.spimpulse.map = 'linear';        % linear, [filename]

opts.thrust.ptest = 101325;           % test pressure [Pa]
opts.thrust.map = 'linear';           % linear, [filename]
opts.thrust.delay = 'Tode1';          % ode1 (1st oder ODE), [function handle]
opts.thrust.offset = [0;0;0];         % line of action of thrust

opts.CP.cone.map = 'analytical';      % dummy, [filename]
opts.CP.cone.type = 'cone';           % body, fin
opts.CP.body.map = 'analytical';      
opts.CP.body.type = 'body';           
opts.CP.fin.map = 'analytical';       
opts.CP.fin.type = 'fin';

opts.polar.cone.map = 'dummy';        % dummy, [filename], cfd
opts.polar.body.map = 'dummy';        % dummy, [filename], cfd
opts.polar.fin.map = 'dummy';         % dummy, [filename], xfoil, cfd

opts.gimbal.delay = 'Gode1';          % ode1 (1st oder ODE), [function handle]

opts.env.altmap = 'dummy';            % dummy, [filename]
opts.env.windmap = 'const';           % const, [filename]
opts.env.wind = [10;0;0];              % onlt for windmap = const
opts.env.gamma = 1.4;
opts.env.R = 287;

opts.ctrlmode = 'dummy';              % dummy, [ctrl function name]

opts.integ.name = 'ExplicitEuler';    % explicit euler, rk4
opts.integ.dt = 1e-4;
opts.integ.iters = 2e4;
opts.integ.initstate.type = 'default';% default, custom
opts.integ.initstate.state = (1:14)'; % customise!

opts.path.point = [0,0,10];
opts.path.r_tol = 0.1;
opts.path.t_tol = 3;

opts.engine.physname = 'Physics';     % [filename]
opts.engine.MLname = 'none';          % none, [filename]



%% Config
% Loading directories
DirSetup();

% Loading rocket data
rocket = rocketInit(opts);

% Loading environment model
[env.alt.gmap, env.alt.tmap, env.alt.rhomap, env.alt.pmap, env.windmap, env.const] = envInit(opts.env);

% Loading engine
engine = engineInit(opts.engine);

% Loading integrator
integ = integInit(opts.integ);

% Loading path
hoverpoint = Hoverpoint(opts.path);


%% Simulations
freedat = Free(rocket, env, engine, integ);
hoverdat = Path(rocket, env, hoverpoint, integ, opts.ctrlmode);
pathdat = Path(rocket, env, path, integ, opts.ctrlmode);
MCdat = MonteCarlo(rocket, env, integ, opts.ctrlmode);


%% Visualising and Analysing Data