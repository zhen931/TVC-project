function [map] = CPFit(opts, geom)
%%%%% Ouputs cp model function handle

% Determining analytical or file
switch opts.map
    case 'analytical'
        % Analytical models: constant
        switch opts.type
            case 'cone'
                map = @(M, alpha) 0.75*geom.L_cone;
                disp(2)
            case 'body'
                map = @(M, alpha) geom.L_cone + 0.5*geom.L_body; 
            case 'fin'
                map = @(M, alpha) geom.x_fin + 0.25*geom.c_fin;
        end

    otherwise
        % add data cp fits
        % mapdat = readtable(opts.map + '.xslx');
        % mapdat = table2struct(mapdat, "ToScalar",true);
        % map = griddedInterpolant(mapdat.M, mapdat.alpha, mapdat.CP);
end
end