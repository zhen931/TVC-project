function [delay] = gimbalFit(opts)
%%%%% Gimbal delay function

% Thrust Delay Function
delayFunc = str2func(opts.delay);
delay = @(g, g_des) delayFunc(g, g_des);
end