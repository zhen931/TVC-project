function [throttle, gimbal_pitch, gimbal_yaw, nose_AoA] = Dummy(state, control)
%%%%% Dummy control, does nothing. For testing.

throttle = 0.5;
gimbal_pitch = 0;
gimbal_yaw = 0;
nose_AoA = 0;

end