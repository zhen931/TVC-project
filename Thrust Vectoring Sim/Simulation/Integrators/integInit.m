function [integ] = integInit(opts)
%%%%% Initalise integrator

% Parameters
integ.integrate = str2func(opts.name);
integ.dt = opts.dt;
integ.iters = 2e4;

% Initial Conditions
switch opts.initstate.type
    case 'custom'
        integ.initstate = opts.state;
    otherwise
        integ.initstate = zeros(17,1);
        integ.initstate(7) = cos(-pi/4); % quaternion
        integ.initstate(9) = sin(-pi/4);
        integ.initstate(15) = 1; % fuel mass fraction
end
end