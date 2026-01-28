function [map, delay, Tmax] = thrustFit(thrustmap, thrustdelay)
%%%%% Ouputs thrust model function handle

% Mapping
switch thrustmap
    case 'linear'
        Throttle = (0:0.01:1)';
        Thrust = (0:1:100)';
        Altitude = linspace(0,0,length(Throttle))';
        mapdat = table(Throttle,Thrust,Altitude, 'VariableNames',{'Throttle','Thrust','Altitude'});
    case 'curve'
        mapdat = readtable('rocketDat.xlsx', 'Sheet','thrust');
end
mapdat = table2struct(mapdat, "ToScalar",true);
Tmax = max(mapdat.Thrust);
mapdat.Thrust = mapdat.Thrust / Tmax;
map = griddedInterpolant(mapdat.Throttle, mapdat.Thrust);

% Thrust Delay Function
delayFunc = str2func(thrustdelay);
delay = @(T, Tcmd) delayFunc(T, Tcmd);
end