function [rocket] = rocketInit(opts)
%%%%% Reads rocket excel sheet and outputs struct

% Geometry
% Note that x coordinate is along the rocket from cg
rocket.geom = geomFit(opts.rocket.dat);

% Mass
rocket.mass = massFit(opts.rocket.dat, opts.mass);

% Inertia
rocket.inertia = inertiaFit(opts.rocket.dat, opts.inertia);

% CP
rocket.aero.cone.CPmap = CPFit(opts.CP.cone, rocket.geom);
rocket.aero.body.CPmap = CPFit(opts.CP.body, rocket.geom);
rocket.aero.fin.CPmap = CPFit(opts.CP.fin, rocket.geom);

% Polars
[rocket.aero.cone.Ctmap, rocket.aero.cone.Cnmap, rocket.aero.cone.Cmmap] = polarFit(opts.polar.cone);
[rocket.aero.body.Ctmap, rocket.aero.body.Cnmap, rocket.aero.body.Cmmap] = polarFit(opts.polar.body);
[rocket.aero.fin.Ctmap, rocket.aero.fin.Cnmap, rocket.aero.fin.Cmmap] = polarFit(opts.polar.fin);

% Thrust
[rocket.T.map, rocket.T.delay, rocket.T.max, rocket.T.ptest] = thrustFit(opts.thrust);

% Specific Impulse
rocket.T.spImpulsemap = impulseFit(opts.spimpulse, rocket.T.map);

% Gimbal Delay
rocket.gimbal.delay = gimbalFit(opts.gimbal);
end