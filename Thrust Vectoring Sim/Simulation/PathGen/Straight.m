function [path] = Straight(startpoint, endpoint, vTarget)
%%%%% Generates a straight flightpath
%%% Inputs:
% Startpoint - start coordinate [x,y,z] in world FoR
% Endpoint - end coordinate [x,y,z] in world FoR
% vTarget - target velocity
%%% Outputs:
% path - object containing:
%           - desired trajectory(t)
%           - desired velocity(t)

% Determining mode
switch vTarget
    case "const"

    case "func"

    otherwise
        disp("Velocity pathing mode not recognised.")
end

% Creating map
path.r = griddedInterpolant();
path.v = griddedInterpolant();

end