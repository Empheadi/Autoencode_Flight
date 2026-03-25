function [omega_dot_est, est] = est_eso_step(mode, omega_meas, ~, P, est)
%EST_ESO_STEP Second-order ESO for angular acceleration estimation.
% Inputs:
%   mode      : 'init' or 'step'
%   omega_meas: (3x1) [rad/s]
%   P         : parameter struct (uses P.est_eso_bw, P.dt)
%   est       : struct with states z1 (3x1), z2 (3x1)
% Outputs:
%   omega_dot_est : (3x1) [rad/s^2]
%   est           : updated state

switch lower(mode)
    case 'init'
        est = struct();
        est.z1 = zeros(3,1);
        est.z2 = zeros(3,1);
        omega_dot_est = zeros(3,1);

    case 'step'
        w_o = P.est_eso_bw;
        beta1 = 2 * w_o;
        beta2 = w_o^2;

        e = omega_meas - est.z1;
        est.z1 = est.z1 + P.dt * (est.z2 + beta1 * e);
        est.z2 = est.z2 + P.dt * (beta2 * e);

        omega_dot_est = est.z2;

    otherwise
        error('est_eso_step:UnknownMode', 'Unknown mode: %s', mode);
end
end
