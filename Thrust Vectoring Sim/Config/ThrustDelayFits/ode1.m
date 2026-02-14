function [dTdt] = ode1(T, Tcmd)
%%%%% Takes map and returns thrust

tau = 0.15;
dTdt = (Tcmd - T) / tau;
end