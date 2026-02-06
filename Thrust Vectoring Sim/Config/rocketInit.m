function [rocket] = rocketInit(rocketname, massmap, inertiamap, cpmap, bodypolarmap, finpolarmap, thrustmap, thrustdelay, fuelburn)
%%%%% Reads rocket excel sheet and outputs struct

% Filename
filename = strcat(rocketname, '.xlsx');

% Geometry
% Note that x coordinate is along the rocket from cg
geomdat = readtable(filename, 'Sheet','geom');
rocket.geom = table2struct(geomdat);

% Mass
rocket.mass = massFit(filename, massmap);

% Inertia
rocket.inertia = inertiaFit(filename, inertiamap, rocket.mass.m_maxfuel);

% Cp
rocket.aero.cpmap = cpFit(cpmap);

% Polars
rocket.aero.bodymap = polarFit(bodypolarmap);
rocket.aero.finmap = polarFit(finpolarmap);

% Thrust
[rocket.T.map, rocket.T.delay, rocket.T.max] = thrustFit(thrustmap, thrustdelay);

% Fuel burn
rocket.mass.burn = burnFit(fuelburn, rocket.T);
end