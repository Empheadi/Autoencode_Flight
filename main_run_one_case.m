% MAIN_RUN_ONE_CASE Run one closed-loop simulation with baseline estimator.
% Pure MATLAB script entry point.

rng(42);

addpath('sim', 'estimators', 'utils');

P = init_params();
estimator = @est_backdiff_lpf_step;
omega_cmd_fun = @(t) [deg2rad(20) * sin(2 * pi * 0.5 * t); 0; 0];

L = run_episode(P, estimator, omega_cmd_fun);

figure('Name', 'Rate tracking');
subplot(2, 1, 1);
plot(L.t, L.omega_cmd(1, :), 'k--', 'LineWidth', 1.2); hold on;
plot(L.t, L.omega_meas(1, :), 'b', 'LineWidth', 1.0);
grid on; xlabel('Time [s]'); ylabel('\omega_x [rad/s]');
legend('\omega_{cmd,x}', '\omega_{meas,x}', 'Location', 'best');
title('Commanded vs measured roll rate');

subplot(2, 1, 2);
plot(L.t, L.omega_dot_true(1, :), 'k', 'LineWidth', 1.2); hold on;
plot(L.t, L.omega_dot_est(1, :), 'r', 'LineWidth', 1.0);
grid on; xlabel('Time [s]'); ylabel('\dot{\omega}_x [rad/s^2]');
legend('True', 'Estimated', 'Location', 'best');
title('True vs estimated roll angular acceleration');
