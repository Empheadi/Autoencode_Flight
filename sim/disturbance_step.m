function [dist, dist_state] = disturbance_step(t, P, dist_state)
%DISTURBANCE_STEP Disturbance torque generator.
% Inputs:
%   t          : (1x1) [s], current time
%   P          : parameter struct
%   dist_state : struct with field x (3x1), filter state for band-limited mode
% Outputs:
%   dist       : (3x1) [N*m], disturbance torque
%   dist_state : updated state struct

if ~isfield(dist_state, 'x') || isempty(dist_state.x)
    dist_state.x = zeros(3,1);
end

switch lower(P.dist_type)
    case 'none'
        dist = zeros(3,1);

    case 'step'
        active = (t > P.T/3) && (t < 2*P.T/3);
        dist = P.dist_std .* double(active);

    case 'band_limited'
        w = 2*pi*P.dist_bw;
        for i = 1:3
            dist_state.x(i) = dist_state.x(i) + P.dt * ( ...
                -w * dist_state.x(i) + w * P.dist_std(i) * randn(1,1));
        end
        dist = dist_state.x;

    case 'sinusoidal'
        dist = P.dist_std .* sin(2*pi*3*t);

    otherwise
        error('disturbance_step:UnknownType', 'Unknown P.dist_type: %s', P.dist_type);
end
end
