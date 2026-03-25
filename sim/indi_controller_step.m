function [delta_cmd, ctrl] = indi_controller_step(omega_meas, omega_cmd, omega_dot_est, delta_prev, P, ctrl)
%INDI_CONTROLLER_STEP One-step INDI rate-loop control law.
% Inputs:
%   omega_meas   : (3x1) [rad/s], measured rates
%   omega_cmd    : (3x1) [rad/s], commanded rates
%   omega_dot_est: (3x1) [rad/s^2], estimated angular acceleration
%   delta_prev   : (3x1) [rad], previous actuator command/position
%   P            : parameter struct
%   ctrl         : controller state struct (reserved)
% Outputs:
%   delta_cmd    : (3x1) [rad], saturated actuator command
%   ctrl         : updated controller state struct

omega_dot_des = P.K_rate * (omega_cmd - omega_meas);
delta_inc = P.B0_ctrl \ (omega_dot_des - omega_dot_est);
delta_cmd = delta_prev + delta_inc;
delta_cmd = max(min(delta_cmd, P.delta_max), -P.delta_max);

if isempty(ctrl)
    ctrl = struct();
end
end
