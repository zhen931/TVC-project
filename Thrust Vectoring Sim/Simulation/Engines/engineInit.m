function [engine] = engineInit(opts)
%%%%% Initalises engine

engine = str2func(opts.physname);

switch opts.MLname
    case "none"
        return
    otherwise
        MLengine = str2func(opts.MLname);
        engine = @(t, rocket, state, control, env) engine(t, rocket, state, control, env) + MLengine(t, rocket, state, control, env);
end
end