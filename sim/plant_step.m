function [S_next, omega_dot_true] = plant_step(S, delta_actual, P, dist)
%PLANT_STEP One-step rigid-body angular dynamics update.
% Inputs:
%   S            : struct with fields
%                  S.omega (3x1) [rad/s], current body rates
%                  S.V     (1x1) [m/s], speed state
%   delta_actual : (3x1) [rad], actuator deflections
%   P            : parameter struct
%   dist         : (3x1) [N*m], disturbance torque
% Outputs:
%   S_next       : struct, next state
%   omega_dot_true : (3x1) [rad/s^2], true angular acceleration

if isfield(S, 'V')
    V = S.V;
else
    V = P.V0;
end

A = P.A0 + P.AV * (V - P.V0); % 3x3
B = P.B0 + P.BV * (V - P.V0); % 3x3

M_aero = A * S.omega + B * delta_actual;                        % [N*m], 3x1 equivalent
M_nl   = -P.mu_nl * (norm(S.omega)^2) * S.omega;                % [N*m], 3x1
M_gyro = cross(S.omega, P.I * S.omega);                         % [N*m], 3x1
M_total = M_aero + M_nl + dist - M_gyro;                        % [N*m], 3x1

omega_dot_true = P.I \ M_total;                                 % [rad/s^2], 3x1

S_next = S;
S_next.omega = S.omega + P.dt * omega_dot_true;
S_next.V = V;
end
