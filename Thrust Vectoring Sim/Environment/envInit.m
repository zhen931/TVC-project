function [gmap, tmap, rhomap, wind] = envInit(envmap)
%%%%% Outputs environment model

% Default [0,0,0] wind
wind = [0, 0, 0];

% Getting Map Data
switch envmap
    case 'dummy'
        envdat.Altitude = (0:100:45000)';
        envdat.T = ones(size(Altitude)) * 273;
        envdat.rho = ones(size(Altitude)) * 1.225;
    otherwise
        envdat = readtable(envmap + '.xlsx');
        envdat = table2struct(envdat, "ToScalar",true);
end

% Mapping T and rho
tmap = griddedInterpolant(envdat.Altitude, envdat.T);
rhomap = griddedInterpolant(envdat.Altitude, envdat.rho);

% Mapping g
r_e = 6378100;
gmap = @(h) 9.81*(r_e / (r_e + h))^2;

end