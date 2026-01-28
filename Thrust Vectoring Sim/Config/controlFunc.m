function [ctrl] = controlFunc(state, control, mode)
%%%%% Control engine. Modular development possible using custom function
%%%%% handles
%%% Inputs:
%%% State vector
%%% Control vector

switch mode
    case 'dummy'
        ctrl = Dummy(state, control);
    case 'pid'
        ctrl = PIDControl(state, control);
    case 'rl'
        ctrl = RLControl(state, control);
    otherwise
        error('Unknown control mode');
end
end