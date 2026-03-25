% MAIN_RUN_ONE_CASE Run one closed-loop simulation case and plot quick diagnostics.
% Global project rules:
% - Fixed seed
% - Fixed-step discrete-time simulation

rng(42);

addpath('sim', 'estimators', 'utils');

P = init_params();

% Example command: roll-rate sinusoid
omega_cmd_fun = @(t) [deg2rad(20) * sin(2*pi*0.5*t); 0; 0];

% Run all classical estimators for a quick comparison
L_backdiff = run_episode(P, @est_backdiff_lpf_step, omega_cmd_fun);
L_eso      = run_episode(P, @est_eso_step,          omega_cmd_fun);
L_comp     = run_episode(P, @est_compfilt_step,     omega_cmd_fun);

figure('Name', 'Rate tracking');
subplot(3,1,1);
plot(L_backdiff.t, L_backdiff.omega_cmd(1,:), 'k--', 'LineWidth', 1.2); hold on;
plot(L_backdiff.t, L_backdiff.omega_meas(1,:), 'b', 'LineWidth', 1.0);
ylabel('\omega_x [rad/s]'); grid on; legend('cmd', 'meas');
title('Roll rate command tracking');

subplot(3,1,2);
plot(L_backdiff.t, L_backdiff.omega_dot_true(1,:), 'k', 'LineWidth', 1.2); hold on;
plot(L_backdiff.t, L_backdiff.omega_dot_est(1,:), 'r');
plot(L_eso.t,      L_eso.omega_dot_est(1,:),      'g');
plot(L_comp.t,     L_comp.omega_dot_est(1,:),     'b');
ylabel('\dot{\omega}_x [rad/s^2]'); grid on;
legend('true', 'backdiff', 'eso', 'compfilt');
title('Acceleration estimate comparison (roll axis)');

subplot(3,1,3);
plot(L_backdiff.t, L_backdiff.delta_true(1,:), 'm');
xlabel('Time [s]'); ylabel('\delta_x [rad]'); grid on;
title('Actuator response (roll channel)');

fprintf('Done. Simulated %d samples at dt=%.4f s.\n', numel(L_backdiff.t), P.dt);
