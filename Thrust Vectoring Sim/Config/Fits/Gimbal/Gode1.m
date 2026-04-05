function [dgdt] = Gode1(g, gcmd)
%%%%% Takes mapped value and returns gimbal rate

tau = 0.02;
dgdt = (gcmd - g) / tau;
end