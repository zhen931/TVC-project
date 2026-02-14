function [inertia] = inertiaFit(filename, inertiamap, m_maxfuel)
%%%%% Initialises inertial parameters

% Inertia values
inertiadat = readtable(filename, 'Sheet','int');
inertiadat = table2struct(inertiadat);
inertia.dry = [inertiadat.I_xx_dry, inertiadat.I_xy_dry, inertiadat.I_xz_dry;
               inertiadat.I_xy_dry, inertiadat.I_yy_dry, inertiadat.I_yz_dry;
               inertiadat.I_xz_dry, inertiadat.I_yz_dry, inertiadat.I_zz_dry];
inertia.wet = [inertiadat.I_xx_wet, inertiadat.I_xy_wet, inertiadat.I_xz_wet;
               inertiadat.I_xy_wet, inertiadat.I_yy_wet, inertiadat.I_yz_wet;
               inertiadat.I_xz_wet, inertiadat.I_yz_wet, inertiadat.I_zz_wet];

% Fitting inertial change with fuel burn
switch inertiamap
    case 'linear'
        inertia.map = @(m_fuel) inertia.wet .* (1 - (m_fuel / m_maxfuel));
    otherwise
        mapdat = readtable(inertiamap + '.xlsx');
        mapdat = table2struct(mapdat, "ToScalar",true);
        inertia.map = griddedInterpolant(mapdat.m_fuel, [mapdat.I_xx, mapdat.I_xy, mapdat.I_xz; ...
                                                         mapdat.I_xy, mapdat.I_yy, mapdat.I_yz; ...
                                                         mapdat.I_xz, mapdat.I_yz, mapdat.I_zz]);
end
end