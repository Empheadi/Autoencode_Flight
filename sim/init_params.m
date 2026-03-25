function P = init_params()
%INIT_PARAMS Initialize simulation parameters for INDI angular-rate loop.
%   Output:
%     P : struct, simulation/controller/estimator parameters (SI units)

% Vehicle / integration
P.I = diag([4000, 6000, 8000]); % [kg*m^2], 3x3 inertia matrix
P.dt = 0.002;                   % [s], fixed step (500 Hz)
P.T  = 30;                      % [s], episode length

% Aerodynamic moment model: M_aero = A(V)*omega + B(V)*delta
P.V0 = 80;                      % [m/s], nominal speed
P.A0 = [-4.0, 0,   0.5; ...
         0,  -3.0, 0;   ...
         0.3, 0,  -1.5];        % [1/s], 3x3
P.AV = P.A0 * 0.005;            % [1/(s*(m/s))], 3x3

P.B0 = [60,  0,   5;  ...
         0, -40,  0;  ...
         3,  0, -20];           % [rad/s^2/rad], 3x3
P.BV = P.B0 * 0.003;            % [rad/s^2/rad/(m/s)], 3x3

% Nonlinear damping term: M_nl = -mu*||omega||^2*omega
P.mu_nl = 0.05;

% Actuator: first-order lag + limits
P.tau_act   = [0.03; 0.03; 0.04];        % [s], 3x1
P.delta_max = deg2rad([25; 20; 25]);     % [rad], 3x1 position limit
P.rate_max  = deg2rad([80; 60; 80]);     % [rad/s], 3x1 rate limit

% Sensor model (gyro)
P.gyro_noise_std = deg2rad([0.5; 0.5; 0.3]);   % [rad/s], 3x1
P.gyro_bias_std  = deg2rad([0.02; 0.02; 0.01]);% [rad/s], 3x1
P.gyro_quantize  = deg2rad(0.01);              % [rad/s], scalar, 0=disabled

% Disturbance torque
P.dist_type = 'band_limited';          % {'none','step','band_limited','sinusoidal'}
P.dist_std  = [5; 5; 3];               % [N*m], 3x1
P.dist_bw   = 10;                      % [Hz], scalar

% Controller
P.K_rate  = diag([15, 12, 8]);         % [1/s], 3x3
P.B0_ctrl = P.B0;                      % [rad/s^2/rad], 3x3

% Estimator parameters
P.est_lpf_alpha = 0.85;                % LPF coefficient
P.est_eso_bw    = 50;                  % [rad/s]
P.est_cf_alpha  = 0.7;                 % blend (0=model, 1=diff)
end
