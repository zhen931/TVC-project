function [geom] = geomFit(filename)
%%%%% Initialise rocket geometry parameters

% Reading file
geomdat = readtable(filename, 'Sheet','geom');
geom = table2struct(geomdat);

% Derived parameters
geom.S_ref = pi * geom.d_ref / 4;
geom.L_ref = geom.L_cone + geom.L_body;
end