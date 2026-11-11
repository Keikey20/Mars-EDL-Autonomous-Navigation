function imu = init_imu(cfg)
imu.bias = cfg.imu.initialBiasStd * randn(3,1);
imu.noiseStd = cfg.imu.accelNoiseStd;
imu.biasWalkStd = cfg.imu.biasWalkStd;
imu.biasJumpRate = cfg.imu.biasJumpRate;
imu.biasJumpStd = cfg.imu.biasJumpStd;
end
