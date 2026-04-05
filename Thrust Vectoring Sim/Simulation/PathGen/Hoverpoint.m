function [hoverpoint] = Hoverpoint(opts)
%%%%% Generates a hoverpoint object for the simulator to target

hoverpoint.point = opts.point;
hoverpoint.r_tol = opts.r_tol;
hoverpoint.t_tol = opts.t_tol;
end