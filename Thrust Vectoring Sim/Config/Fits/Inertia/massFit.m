function [mass] = massFit(filename, opts)
%%%%% Ouputs mass model function handle

% Mass values
mdat = readtable(filename, 'Sheet','m');
mdat = table2struct(mdat);
mass.m_dry = mdat.m_dry;
mass.m_wet = mdat.m_wet;
mass.m_maxfuel = mass.m_wet - mass.m_dry;
mass.cg_dry = [mdat.cgx_dry; mdat.cgy_dry; mdat.cgz_dry];
mass.cg_wet = [mdat.cgx_wet; mdat.cgy_wet; mdat.cgz_wet];

% Fitting cg change with normalised fuel fraction
switch opts.map
    case 'linear'
        mass.map = @(m_fuel_norm) mass.cg_dry + (mass.cg_wet - mass.cg_dry) .* m_fuel_norm;
    otherwise
        mapdat = readtable(massmap + '.xlsx');
        mapdat = table2struct(mapdat, "ToScalar",true);
        mass.map = griddedInterpolant(mapdat.m_fuel_norm, [mapdat.cg_x, mapdat.cg_y, mapdat.cg_z]);
end
end