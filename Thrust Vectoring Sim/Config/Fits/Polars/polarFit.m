function [Ctmap, Cnmap, Cmmap] = polarFit(opts)
%%%%% Outputs polar map

% Getting body polar data
switch opts.map
    case 'dummy'
        [mapdat.M, mapdat.alpha] = ndgrid((0:0.01:1)', (0:0.01:1)');
        Ct = ones(size(mapdat.M)) * 0.5;
        Cn = ones(size(Ct)) * 0.5;
        Cm = ones(size(Ct)) * 0.1;
    case 'xfoil'
        disp("XFoil not yet implemented.")
    case 'cfd'
        disp("CFD Not yet implemented.")
    otherwise
        polardat = readtable(opts.map + '.xlsx');
        polardat = table2struct(polardat, "ToScalar",true);
        % figure out how to handle data
end

% Building maps
% map = griddedInterpolant(mapdat.AoA, mapdat.Thrust);
Ctmap = griddedInterpolant(mapdat.M, mapdat.alpha, Ct);
Cnmap = griddedInterpolant(mapdat.M, mapdat.alpha, Cn);
Cmmap = griddedInterpolant(mapdat.M, mapdat.alpha, Cm);
end