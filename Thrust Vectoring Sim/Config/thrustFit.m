function [map, delay, burn, Tmax] = thrustFit(thrustmap, thrustdelay, fuelburn)
%%%%% Ouputs thrust model function handle

% Mapping
switch thrustmap
    case 'linear'
        mapdat.Throttle = (0:0.01:1)';
        mapdat.Thrust = (0:1:100)';
        mapdat.Altitude = linspace(0,0,length(mapdat.Throttle))'; % for now thrust does not vary with altitude
    otherwise
        mapdat = readtable(thrustmap + '.xslx');
        mapdat = table2struct(mapdat, "ToScalar",true);
end
Tmax = max(mapdat.Thrust);
mapdat.Thrust = mapdat.Thrust / Tmax;
map = griddedInterpolant(mapdat.Throttle, mapdat.Thrust);

% Thrust Delay Function
delayFunc = str2func(thrustdelay);
delay = @(T, Tcmd) delayFunc(T, Tcmd); 

% need to find a way to decouple the input params from the function definitions. 
% delay = @(T, Tcmd, state) delayFunc(T, Tcmd, state, rocket) ???
end