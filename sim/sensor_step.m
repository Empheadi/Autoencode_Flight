function omega_meas = sensor_step(omega_true, bias, P)
%SENSOR_STEP Gyro measurement with bias, white noise, and quantization.
% Inputs:
%   omega_true : (3x1) [rad/s], true body rate
%   bias       : (3x1) [rad/s], fixed/slow gyro bias
%   P          : parameter struct
% Output:
%   omega_meas : (3x1) [rad/s], measured body rate

noise = P.gyro_noise_std .* randn(3,1);
omega_meas = omega_true + bias + noise;

if P.gyro_quantize > 0
    omega_meas = round(omega_meas / P.gyro_quantize) * P.gyro_quantize;
end
end
