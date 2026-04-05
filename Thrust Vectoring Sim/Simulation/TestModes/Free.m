function [data] = Free(rocket, env, engine, integ)
%%%%% General rocket flight test

%% Simulation
% Initalising state
state = integ.initstate;

% Initialising output
data = dataInit(integ.iters);

% Hardcoded no control
control.throttle = 1;
control.g_yaw = 0;
control.g_pitch = 0;

% Simulation loop
for iter = 1:integ.iters
    t = iter * integ.dt;
    data.t(iter) = t;
    
    [state_dot, debug] = engine(t, rocket, state, control, env, true);
    state = integ.integrate(state, state_dot, integ.dt);
    
    data.state(iter,:) = state';
    data.state_dot(iter,:) = state_dot;
    % data.debug(iter,:) = debug;

    % Writing to command line
    if (mod(iter,10000)) == 0
        disp("Iter No.: " + iter)
        disp("Thrust: " + state(14))
        disp("Fuel Ratio: " + state(15) + " | RoBurn: " + state_dot(15))
        disp("r: " + state(1) + ", " + state(2) + ", " + state(3))
        disp("")
    end
    
end

% Saving data
save("simresults.mat","data")


% Displaying history
opts.labels = {
    'Altitude',  @(k) data.state(k, 3),                '%.1f',   'm';
    'Speed',     @(k) norm(data.state(k, 4:6)),         '%.2f',   'm/s';
    'Fuel',      @(k) data.state(k, 15) * 100,         '%.1f',   '%';
    'Thrust',    @(k) data.state(k, 14),               '%.1f',   'N';
};

opts.playback_speed  = 20;
opts.dt              = integ.dt;
opts.orient_scale    = 2;
opts.cam_pad         = 10;

animate(data, opts);
