function [dist, dist_state] = disturbance_step(t, P, dist_state)
%DISTURBANCE_STEP Generate disturbance torque at simulation time t.
%   [dist, dist_state] = disturbance_step(t, P, dist_state)
%
% Inputs:
%   t          [1x1] Time [s]
%   P          [struct] Parameter struct
%   dist_state [struct] Internal disturbance generator state
%
% Outputs:
%   dist       [3x1] Disturbance torque [N*m]
%   dist_state [struct] Updated disturbance state

switch lower(P.dist_type)
    case 'none'
        dist = zeros(3, 1);

    case 'step'
        isActive = (t > P.T / 3) && (t < 2 * P.T / 3);
        dist = P.dist_std .* double(isActive);

    case 'band_limited'
        if ~isfield(dist_state, 'x') || isempty(dist_state.x)
            dist_state.x = zeros(3, 1);
        end
        w = 2 * pi * P.dist_bw;
        for i = 1:3
            dist_state.x(i) = dist_state.x(i) + P.dt * (-w * dist_state.x(i) + w * P.dist_std(i) * randn);
        end
        dist = dist_state.x;

    case 'sinusoidal'
        dist = P.dist_std .* sin(2 * pi * 3 * t);

    otherwise
        error('disturbance_step:BadType', 'Unknown disturbance type: %s', P.dist_type);
end
end
