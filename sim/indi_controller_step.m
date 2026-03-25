function [delta_cmd, ctrl] = indi_controller_step(omega_meas, omega_cmd, omega_dot_est, delta_prev, P, ctrl)
%INDI_CONTROLLER_STEP One-step INDI rate-loop update.
%   [delta_cmd, ctrl] = indi_controller_step(omega_meas, omega_cmd, omega_dot_est, delta_prev, P, ctrl)
%
% Inputs:
%   omega_meas   [3x1] Measured angular rate [rad/s]
%   omega_cmd    [3x1] Commanded angular rate [rad/s]
%   omega_dot_est [3x1] Estimated angular acceleration [rad/s^2]
%   delta_prev   [3x1] Previous actuator command/position [rad]
%   P            [struct] Parameter struct
%   ctrl         [struct] Controller state (placeholder)
%
% Outputs:
%   delta_cmd    [3x1] Commanded actuator deflection [rad]
%   ctrl         [struct] Updated controller state

omega_dot_des = P.K_rate * (omega_cmd - omega_meas);
delta_inc = P.B0_ctrl \ (omega_dot_des - omega_dot_est);
delta_cmd = delta_prev + delta_inc;
delta_cmd = min(max(delta_cmd, -P.delta_max), P.delta_max);

if nargin < 6 || isempty(ctrl)
    ctrl = struct();
end
end
