function [omega_dot_est, est] = est_compfilt_step(mode, omega_meas, delta, P, est)
%EST_COMPFILT_STEP Complementary angular-acceleration estimator.
% Inputs:
%   mode      : 'init' or 'step'
%   omega_meas: (3x1) [rad/s]
%   delta     : (3x1) [rad], actuator state/command used by model branch
%   P         : parameter struct
%   est       : state struct with omega_prev, omega_dot_filt
% Outputs:
%   omega_dot_est : (3x1) [rad/s^2]
%   est           : updated state

switch lower(mode)
    case 'init'
        est = struct();
        est.omega_prev = zeros(3,1);
        est.omega_dot_filt = zeros(3,1);
        omega_dot_est = zeros(3,1);

    case 'step'
        omega_dot_diff = (omega_meas - est.omega_prev) / P.dt;
        est.omega_prev = omega_meas;

        V = P.V0;
        A = P.A0 + P.AV * (V - P.V0);
        B = P.B0_ctrl + P.BV * (V - P.V0);
        omega_dot_model = P.I \ (A * omega_meas + B * delta - cross(omega_meas, P.I * omega_meas));

        omega_dot_raw = P.est_cf_alpha * omega_dot_diff + (1 - P.est_cf_alpha) * omega_dot_model;
        est.omega_dot_filt = P.est_lpf_alpha * est.omega_dot_filt + ...
                             (1 - P.est_lpf_alpha) * omega_dot_raw;
        omega_dot_est = est.omega_dot_filt;

    otherwise
        error('est_compfilt_step:UnknownMode', 'Unknown mode: %s', mode);
end
end
