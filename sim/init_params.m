function P = init_params()
%INIT_PARAMS Build simulation parameter struct.
%   P = init_params()
%
% Outputs:
%   P [struct] Simulation, plant, actuator, sensor, disturbance, and controller parameters.

% Vehicle and integration setup
P.I = diag([4000, 6000, 8000]);          % [kg*m^2] inertia matrix
P.dt = 0.002;                             % [s] fixed step (500 Hz)
P.T = 30;                                 % [s] episode duration

% Aerodynamic model scheduling
P.V0 = 80;                                % [m/s] nominal speed
P.A0 = [-4.0, 0.0, 0.5; ...
         0.0, -3.0, 0.0; ...
         0.3, 0.0, -1.5];                 % [1/s]
P.AV = P.A0 * 0.005;                      % [1/(s*(m/s))]

P.B0 = [60, 0, 5; ...
        0, -40, 0; ...
        3, 0, -20];                       % [rad/s^2/rad]
P.BV = P.B0 * 0.003;                      % [rad/s^2/rad/(m/s)]

% Mild nonlinear damping term M_nl = -mu * ||omega||^2 * omega
P.mu_nl = 0.05;

% Actuator dynamics and limits
P.tau_act = [0.03; 0.03; 0.04];           % [s]
P.delta_max = deg2rad([25; 20; 25]);      % [rad]
P.rate_max = deg2rad([80; 60; 80]);       % [rad/s]

% Gyro sensor model
P.gyro_noise_std = deg2rad([0.5; 0.5; 0.3]);
P.gyro_bias_std = deg2rad([0.02; 0.02; 0.01]);
P.gyro_quantize = deg2rad(0.01);          % [rad], 0 disables quantization

% Disturbance torque model
P.dist_type = 'band_limited';             % {'none','step','band_limited','sinusoidal'}
P.dist_std = [5; 5; 3];                   % [N*m]
P.dist_bw = 10;                           % [Hz]

% Controller and estimator defaults
P.K_rate = diag([15, 12, 8]);
P.B0_ctrl = P.B0;

P.est_lpf_alpha = 0.85;
P.est_eso_bw = 50;                        % [rad/s]
P.est_cf_alpha = 0.7;
end
