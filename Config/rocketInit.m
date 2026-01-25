function [rocket] = rocketInit(thrustmode, thrustfit)
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
switch thrustmode
    case 'linear'
        Time = (0:0.1:10)';
        Throttle = (0:0.01:1)';
        Thrust = (0:1:100)';
        Altitude = linspace(0,0,length(Time))';
        Tdat = table(Time,Throttle,Thrust,Altitude, 'VariableNames',{'Time','Throttle','Thrust','Altitude'});
    case 'curve'
        Tdat = readtable('rocketDat.xlsx', 'Sheet','thrust');
end
rocket.T.dat = table2struct(Tdat, "ToScalar",true);

% Thrust Fitting
rocket.T.Tmax = max(rocket.T.Tdat.Thrust);
rocket.T.dat.normThrust = rocket.T.Thrust / rocket.T.Tmax;
handle = str2func(thrustfit);
rocket.T.map = handle(rocket.T.dat);

end