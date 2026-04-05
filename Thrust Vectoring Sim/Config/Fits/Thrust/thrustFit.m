function [map, delay, Tmax, ptest] = thrustFit(opts)
%%%%% Ouputs thrust model function handle

% Test Pressure
ptest = opts.ptest;

% Mapping
switch opts.map
    case 'linear'
        mapdat.Throttle = (0:0.01:1)';
        mapdat.Thrust = linspace(0, 1000, length(mapdat.Throttle))';
    otherwise
        mapdat = readtable(opts.map + '.xslx');
        mapdat = table2struct(mapdat, "ToScalar",true);
end
Tmax = max(mapdat.Thrust);
Thrust_norm = mapdat.Thrust / Tmax;
map = griddedInterpolant(mapdat.Throttle, Thrust_norm);

% Thrust Delay Function
delayFunc = str2func(opts.delay);
delay = @(T, Tcmd) delayFunc(T, Tcmd);
end