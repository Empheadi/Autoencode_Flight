function [S_next, omega_dot_true] = plant_step(S, delta_actual, P, dist)
%PLANT_STEP Advance rigid-body angular-rate state by one fixed step.
%   [S_next, omega_dot_true] = plant_step(S, delta_actual, P, dist)
%
% Inputs:
%   S            [struct] State with fields:
%                .omega [3x1] angular rate [rad/s]
%                .V     [1x1] speed [m/s]
%   delta_actual [3x1] Actuator deflection [rad]
%   P            [struct] Parameter struct
%   dist         [3x1] Disturbance torque [N*m]
%
% Outputs:
%   S_next       [struct] Next state (same fields as S)
%   omega_dot_true [3x1] True angular acceleration [rad/s^2]

V = S.V;
A = P.A0 + P.AV * (V - P.V0);
B = P.B0 + P.BV * (V - P.V0);

M_aero = A * S.omega + B * delta_actual;
M_nl = -P.mu_nl * (norm(S.omega)^2) * S.omega;
M_gyro = cross(S.omega, P.I * S.omega);
M_total = M_aero + M_nl + dist - M_gyro;

omega_dot_true = P.I \ M_total;

S_next = S;
S_next.omega = S.omega + P.dt * omega_dot_true;
S_next.V = S.V;
end
