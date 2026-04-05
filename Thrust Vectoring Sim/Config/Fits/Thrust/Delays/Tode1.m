function [dTdt] = Tode1(T, Tcmd)
%%%%% Takes mapped value and returns thrust

tau = 0.15;
dTdt = (Tcmd - T) / tau;
end