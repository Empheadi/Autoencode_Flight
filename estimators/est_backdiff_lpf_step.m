function [omega_dot_est, est] = est_backdiff_lpf_step(mode, omega_meas, ~, P, est)
%EST_BACKDIFF_LPF_STEP Backward-difference + low-pass angular acceleration estimator.
% Inputs:
%   mode      : 'init' or 'step'
%   omega_meas: (3x1) [rad/s], measured rate (used in 'step')
%   P         : parameter struct
%   est       : estimator state struct
% Outputs:
%   omega_dot_est : (3x1) [rad/s^2]
%   est           : updated estimator state

switch lower(mode)
    case 'init'
        est = struct();
        est.omega_prev = zeros(3,1);
        est.omega_dot_filt = zeros(3,1);
        omega_dot_est = zeros(3,1);

    case 'step'
        omega_dot_raw = (omega_meas - est.omega_prev) / P.dt;
        est.omega_dot_filt = P.est_lpf_alpha * est.omega_dot_filt + ...
                             (1 - P.est_lpf_alpha) * omega_dot_raw;
        omega_dot_est = est.omega_dot_filt;
        est.omega_prev = omega_meas;

    otherwise
        error('est_backdiff_lpf_step:UnknownMode', 'Unknown mode: %s', mode);
end
end
