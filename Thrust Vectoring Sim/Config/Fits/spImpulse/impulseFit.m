function [map] = impulseFit(opts, Tmap)
%%%%% Outputs specific impulse map. Note that this is at sea level

% Mapping
switch opts.map
    case 'linear'
        mapdat.Throttle = (0.01:0.01:1)';
        mapdat.mdot = linspace(0.000001,0.0001, length(mapdat.Throttle))';
    otherwise
        mapdat = readtable(opts.map + '.xslx');
        mapdat = table2struct(mapdat, "ToScalar",true);
end
% Getting thrust data
thrustdata = Tmap(mapdat.Throttle);

% Calculating I at Sea Level
Wdot = mapdat.mdot * 9.81;
spImpulse_sl = thrustdata ./ Wdot;

% Creating map
map = griddedInterpolant(mapdat.Throttle, spImpulse_sl);
end