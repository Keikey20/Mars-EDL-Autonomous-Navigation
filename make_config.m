function cfg = make_config()
% Simplified, self-contained Mars EDL navigation simulation.
cfg.dt = 1.0;
cfg.T = 300;
cfg.Nmc = 100;                 % Increase to 1000+ after validation
cfg.seed = 42;

cfg.mars.mu = 4.282837e13;     % m^3/s^2
cfg.mars.radius = 3389.5e3;    % m
cfg.atm.rho0 = 0.020;          % kg/m^3 (simplified nominal value)
cfg.atm.scaleHeight = 11.1e3;  % m
cfg.vehicle.CdA_over_m = 0.0015; % m^2/kg, simplified effective value
cfg.atm.rhoScaleSigma = 0.05;

cfg.entry.altitude = 125e3;    % m
cfg.entry.speed = 5500;        % m/s
cfg.entry.gammaDeg = -12;      % negative = descending

cfg.imu.accelNoiseStd = 0.01;  % m/s^2 per axis (illustrative)
cfg.imu.biasWalkStd = 2e-4;    % m/s^2/sqrt(s)
cfg.imu.initialBiasStd = 0.02; % m/s^2
cfg.imu.biasJumpRate = 1e-3;   % events/s
cfg.imu.biasJumpStd = 0.01;    % m/s^2

cfg.clock.drift = 1e-6;        % s/s; deliberately visible demonstration value
cfg.clock.jitterStd = 1e-7;    % s per update

cfg.trn.basePositionNoise = 5; % m, 1-sigma
cfg.trn.confidenceThreshold = 0.30;
cfg.trn.kDust = 0.8;
cfg.trn.measurementRateHz = 1;

cfg.filter.initialPositionStd = 30; % m
cfg.filter.initialVelocityStd = 2;  % m/s
cfg.filter.initialBiasStd = 0.05;   % m/s^2
cfg.filter.baselineTrnStd = 10;     % m
cfg.filter.enhancedMinTrnStd = 5;   % m
cfg.filter.enhancedMaxTrnStd = 100; % m
end
