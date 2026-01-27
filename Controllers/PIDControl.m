function [ctrl] = PIDControl(state, control)
% PID controller for TVC
% inputs:
% state - struct containing q, omega, time
% control - previous control state (unused here)
    % GAIN SETTINGS & LIMITS
    % Proportional (P): Reaction to current angle error
    % Integral (I): Correction for steady-state drift (e.g. wind)
    % Derivative (D): Damping to prevent oscillation
    Kp = 2.5;  
    Ki = 0.5;  
    Kd = 0.8;  
    
    % Physical Gimbal Limit (degrees -> radians)
    maxGimbalAngle = 5 * (pi / 180); 
    
    if isempty(integralError)
        integralError = [0; 0];
        lastTime = 0;
    end
    
    % Handle first time step
    if state.time == 0
        ctrl = [0; 0];
        return;
    end

    dt = state.time - lastTime;
    
    % Safety check for very small dt (division by zero protection)
    if dt < 1e-5
        dt = 1e-5;
    end

    % Convert Quaternion to Tilt Angles
    % Assuming Z is UP along the rocket body
    % We need the angle of the rocket relative to the vertical World Z-axis
    
    q = state.q; % [w, x, y, z]
    
    % Calculate the "Tilt" vector (World Z-axis represented in Body Frame)
    % This tells us: "Where is the ground relative to the rocket nose?"
    % derived from Rotation Matrix column 3
    vx = 2 * (q(2)*q(4) - q(1)*q(3));
    vy = 2 * (q(3)*q(4) + q(1)*q(2));
    % vz = 1 - 2*(q(2)^2 + q(3)^2); % Not needed for tilt
    
    % Small angle approximation: sin(theta) ~= theta
    % This gives us the error in radians from vertical
    pitchError = vx; % Rotation required around Y
    yawError = vy;   % Rotation required around X
    
    currentError = [pitchError; yawError];

    % P Term
    pTerm = Kp * currentError;
    
    % I Term (Accumulate error * time)
    % Anti-windup: Only integrate if error is small (within 10 degrees)
    if abs(pitchError) < (10 * pi/180)
        integralError = integralError + (currentError * dt);
    end
    iTerm = Ki * integralError;
    
    % D Term (Use Gyroscope rates directly for cleaner derivative)
    % state.omega is [omega_x; omega_y; omega_z]
    % We need rates around X and Y
    dTerm = Kd * (-state.omega(1:2));

    % Calculate raw command
    rawCmd = pTerm + iTerm + dTerm;
    
    % Clamp output to physical servo limits
    ctrl = min(max(rawCmd, -maxGimbalAngle), maxGimbalAngle);

    lastTime = state.time;

end