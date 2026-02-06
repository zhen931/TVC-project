function [map] = burnFit(fuelburnmap, T)
%%%%% Outputs mass burn model function handle

% Mapping
switch fuelburnmap % fix all of this
    case 'linear'
        dm_fuel_max = 0.1;
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

end