function delta_next = actuator_step(delta, delta_cmd, P)
%ACTUATOR_STEP First-order actuator lag with rate and position limits.
%   delta_next = actuator_step(delta, delta_cmd, P)
%
% Inputs:
%   delta     [3x1] Current actuator position [rad]
%   delta_cmd [3x1] Commanded actuator position [rad]
%   P         [struct] Parameter struct
%
% Output:
%   delta_next [3x1] Updated actuator position [rad]

delta_dot = (delta_cmd - delta) ./ P.tau_act;
delta_dot = min(max(delta_dot, -P.rate_max), P.rate_max);

delta_next = delta + P.dt * delta_dot;
delta_next = min(max(delta_next, -P.delta_max), P.delta_max);
end
