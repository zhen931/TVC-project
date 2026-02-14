function [map] = polarFit(polarmap)
%%%%% Outputs polar map

% Getting body polar data
switch polarmap
    case 'dummy'
        mapdat.M = (0:0.01:1)';
        mapdat.alpha = (0:0.01:1)';
        mapdat.Re = (0:1:100)';
        mapdat.beta = (0:0.01:1)';
        Ct = ndgrid(mapdat.M, ...
                    mapdat.alpha, ...
                    mapdat.Re, ...
                    mapdat.beta);
        Cn = zeros(size(Ct));
        Cm = zeros(size(Ct));
    case 'xfoil'
        disp("XFoil not yet implemented.")
    case 'cfd'
        disp("CFD Not yet implemented.")
    otherwise
        polardat = readtable(polarmap + '.xlsx');
        polardat = table2struct(polardat, "ToScalar",true);
end
% map = griddedInterpolant(mapdat.AoA, mapdat.Thrust);
map = @(M, alpha, Re, beta) 1;
end