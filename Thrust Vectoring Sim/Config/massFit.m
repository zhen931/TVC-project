function [mass] = massFit(filename, massmap)
%%%%% Ouputs mass model function handle

% Mass values
mdat = readtable(filename, 'Sheet','m');
mdat = table2struct(mdat);
mass.m_dry = mdat.m_dry;
mass.m_wet = mdat.m_wet;
mass.m_maxfuel = mass.m_wet - mass.m_dry;
mass.cg_dry = [mdat.cgx_dry; mdat.cgy_dry; mdat.cgz_dry];
mass.cg_wet = [mdat.cgx_wet; mdat.cgy_wet; mdat.cgz_wet];

% Fitting cg change with fuel burn
switch massmap
    case 'linear'
        mass.map = @(m_fuel) mass.cg_wet .* (1 - (m_fuel / mass.m_maxfuel));
    otherwise
        mapdat = readtable(massmap + '.xlsx');
        mapdat = table2struct(mapdat, "ToScalar",true);
        mass.map = griddedInterpolant(mapdat.m_fuel, [mapdat.cg_x, mapdat.cg_y, mapdat.cg_z]);
end
end