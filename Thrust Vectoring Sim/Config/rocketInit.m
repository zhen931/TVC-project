function [rocket] = rocketInit(thrustmap, thrustdelay)
%%%%% Reads rocket excel sheet and outputs struct

% Geometry
% Note that x coordinate is along the rocket from cg
geomdat = readtable('rocketDat.xlsx', 'Sheet','geom');
rocket.geom = table2struct(geomdat);

% Aero Params (Cp locations, not polars)
aerodat = readtable('rocketDat.xlsx', 'Sheet','aero');
rocket.aero = table2struct(aerodat);

% Interial
inertiadat = readtable('rocketDat.xlsx', 'Sheet','int');
rocket.inertia = table2struct(inertiadat);

% Thrust Data
[rocket.T.map, rocket.T.delay, rocket.T.max] = thrustFit(thrustmap, thrustdelay);
end