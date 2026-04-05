function [data] = dataInit(iters)
%%%%% Initialises output data for a given sim

data.t = zeros(iters, 1);
data.state = zeros(iters, 17);
data.state_dot = zeros(iters, 17);
data.debug = zeros(iters, 18);
end