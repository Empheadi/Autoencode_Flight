function omega_meas = sensor_step(omega_true, bias, P)
%SENSOR_STEP Generate noisy gyro measurement with optional quantization.
%   omega_meas = sensor_step(omega_true, bias, P)
%
% Inputs:
%   omega_true [3x1] True angular rate [rad/s]
%   bias       [3x1] Fixed gyro bias [rad/s]
%   P          [struct] Parameter struct
%
% Output:
%   omega_meas [3x1] Measured angular rate [rad/s]

noise = P.gyro_noise_std .* randn(3, 1);
omega_meas = omega_true + bias + noise;

if P.gyro_quantize > 0
    omega_meas = round(omega_meas / P.gyro_quantize) * P.gyro_quantize;
end
end
