function [imu, aMeas] = sample_imu(imu, x, env, cfg, dt)
% Accelerometer measurement in the simplified aligned frame.
if rand < imu.biasJumpRate*dt
    imu.bias = imu.bias + imu.biasJumpStd*randn(3,1);
end
imu.bias = imu.bias + imu.biasWalkStd*sqrt(dt)*randn(3,1);
aTrue = truth_specific_force(x, env, cfg);
aMeas = aTrue + imu.bias + imu.noiseStd*randn(3,1);
end
