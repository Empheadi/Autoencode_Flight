function [omega_dot_est, est] = est_backdiff_lpf_step(mode, omega_meas, ~, P, est)
%EST_BACKDIFF_LPF_STEP Backward difference with 1st-order LPF.
%   [omega_dot_est, est] = est_backdiff_lpf_step(mode, omega_meas, delta, P, est)
%
% Inputs:
%   mode      [char] 'init' or 'step'
%   omega_meas [3x1] Measured angular rate [rad/s]
%   P         [struct] Parameter struct
%   est       [struct] Estimator state
%
% Outputs:
%   omega_dot_est [3x1] Estimated angular acceleration [rad/s^2]
%   est           [struct] Updated estimator state

switch lower(mode)
    case 'init'
        est = struct();
        est.omega_prev = zeros(3, 1);
        est.omega_dot_filt = zeros(3, 1);
        omega_dot_est = zeros(3, 1);

    case 'step'
        omega_dot_raw = (omega_meas - est.omega_prev) / P.dt;
        est.omega_dot_filt = P.est_lpf_alpha * est.omega_dot_filt + (1 - P.est_lpf_alpha) * omega_dot_raw;
        omega_dot_est = est.omega_dot_filt;
        est.omega_prev = omega_meas;

    otherwise
        error('est_backdiff_lpf_step:BadMode', 'Unknown mode: %s', mode);
end
end
