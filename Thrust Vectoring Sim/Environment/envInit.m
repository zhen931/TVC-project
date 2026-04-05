function [gmap, tmap, rhomap, pmap, windmap, const] = envInit(opts)
%%%%% Outputs environment model

% Getting Wind Model
switch opts.windmap
    case 'const'
        windmap = @(alt, t) opts.wind;
    otherwise
        windfunc = str2func(opts.windmap);
        windmap = @(alt, t) windfunc(alt, t);
end

% Getting Altitude Model
switch opts.altmap
    case 'dummy'
        envdat.Altitude = (0:100:45000)';
        envdat.T = ones(size(envdat.Altitude)) * 273;
        envdat.rho = ones(size(envdat.Altitude)) * 1.225;
        envdat.p = ones(size(envdat.Altitude)) * 101325;
    otherwise
        envdat = readtable(envmap + '.xlsx');
        envdat = table2struct(envdat, "ToScalar",true);
end

% Mapping T, rho, p
tmap = griddedInterpolant(envdat.Altitude, envdat.T);
rhomap = griddedInterpolant(envdat.Altitude, envdat.rho);
pmap = griddedInterpolant(envdat.Altitude, envdat.p);

% Mapping g
r_e = 6378100;
gmap = @(h) 9.81*(r_e / (r_e + h))^2;

% Constants
const.gamma = opts.gamma;
const.R = opts.R;
end