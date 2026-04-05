function [state] = ExplicitEuler(state, state_dot, dt)
%%%%% Integrate (step) engine state derivative with Explicit Euler
state = state + state_dot * dt;
end