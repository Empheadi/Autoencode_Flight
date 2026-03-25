function L = run_episode(P, estimator_handle, omega_cmd_fun)
%RUN_EPISODE Run one closed-loop simulation episode.
%   L = run_episode(P, estimator_handle, omega_cmd_fun)
%
% Inputs:
%   P                [struct] Parameter struct
%   estimator_handle [function_handle] Estimator API:
%                    init: est = f('init', [], [], P, [])
%                    step: [omega_dot_est, est] = f('step', omega_meas, delta, P, est)
%   omega_cmd_fun    [function_handle] omega_cmd_fun(t)->[3x1] [rad/s]
%
% Output:
%   L [struct] Logged time-series arrays

S = struct();
S.omega = zeros(3, 1);
S.V = P.V0;

delta = zeros(3, 1);
bias = P.gyro_bias_std .* randn(3, 1);

est_state = estimator_handle('init', [], [], P, []);
ctrl = struct();
dist_state = struct('x', zeros(3, 1));

N = round(P.T / P.dt);
L = struct();
L.t = zeros(1, N);
L.omega_true = zeros(3, N);
L.omega_meas = zeros(3, N);
L.omega_dot_true = zeros(3, N);
L.omega_dot_est = zeros(3, N);
L.delta_cmd = zeros(3, N);
L.delta_true = zeros(3, N);
L.omega_cmd = zeros(3, N);
L.dist = zeros(3, N);

for k = 1:N
    t = (k - 1) * P.dt;
    omega_cmd = omega_cmd_fun(t);
    omega_meas = sensor_step(S.omega, bias, P);
    [dist, dist_state] = disturbance_step(t, P, dist_state);

    [omega_dot_est, est_state] = estimator_handle('step', omega_meas, delta, P, est_state);
    [delta_cmd, ctrl] = indi_controller_step(omega_meas, omega_cmd, omega_dot_est, delta, P, ctrl);
    delta = actuator_step(delta, delta_cmd, P);
    [S, omega_dot_true] = plant_step(S, delta, P, dist);

    L.t(k) = t;
    L.omega_true(:, k) = S.omega;
    L.omega_meas(:, k) = omega_meas;
    L.omega_dot_true(:, k) = omega_dot_true;
    L.omega_dot_est(:, k) = omega_dot_est;
    L.delta_cmd(:, k) = delta_cmd;
    L.delta_true(:, k) = delta;
    L.omega_cmd(:, k) = omega_cmd;
    L.dist(:, k) = dist;
end
end
