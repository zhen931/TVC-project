function [inertia] = inertiaFit(filename, opts)
%%%%% Initialises inertial parameters

% Inertia values
inertiadat = readtable(filename, 'Sheet','int');
inertiadat = table2struct(inertiadat);
% Dry moment of inertia
inertia.dry = [inertiadat.I_xx_dry, inertiadat.I_xy_dry, inertiadat.I_xz_dry;
               inertiadat.I_xy_dry, inertiadat.I_yy_dry, inertiadat.I_yz_dry;
               inertiadat.I_xz_dry, inertiadat.I_yz_dry, inertiadat.I_zz_dry];
% Wet (fully fuelled) moment of inertia
inertia.wet = [inertiadat.I_xx_wet, inertiadat.I_xy_wet, inertiadat.I_xz_wet;
               inertiadat.I_xy_wet, inertiadat.I_yy_wet, inertiadat.I_yz_wet;
               inertiadat.I_xz_wet, inertiadat.I_yz_wet, inertiadat.I_zz_wet];

% Fitting inertial change with normalised fuel fraction
switch opts.map
    case 'linear'
        inertia.map = @(m_fuel_norm) inertia.dry + (inertia.wet - inertia.dry) .* m_fuel_norm;
    otherwise
        mapdat = readtable(opts.map + '.xlsx');
        mapdat = table2struct(mapdat, "ToScalar",true);
        inertia.map = griddedInterpolant(mapdat.m_fuel_norm, [mapdat.I_xx, mapdat.I_xy, mapdat.I_xz; ...
                                                              mapdat.I_xy, mapdat.I_yy, mapdat.I_yz; ...
                                                              mapdat.I_xz, mapdat.I_yz, mapdat.I_zz]);
end
end