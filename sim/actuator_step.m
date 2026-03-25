function delta_next = actuator_step(delta, delta_cmd, P)
%ACTUATOR_STEP First-order actuator lag with rate and position saturation.
% Inputs:
%   delta     : (3x1) [rad], current actuator position
%   delta_cmd : (3x1) [rad], commanded actuator position
%   P         : parameter struct
% Output:
%   delta_next: (3x1) [rad], next actuator position

delta_dot = (delta_cmd - delta) ./ P.tau_act;                                       % [rad/s]
delta_dot = max(min(delta_dot, P.rate_max), -P.rate_max);                           % rate sat
delta_next = delta + P.dt * delta_dot;
delta_next = max(min(delta_next, P.delta_max), -P.delta_max);                       % pos sat
end
