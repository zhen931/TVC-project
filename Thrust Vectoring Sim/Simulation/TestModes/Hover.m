function [data] = Hover(rocket, env, engine, hoverpoint, integ, ctrlmode)
%%%%% General rocket flight test

%% Simulation
state = initstate;

for i = 1:integ.iters
    state = engine(rocket, state, env, )

end