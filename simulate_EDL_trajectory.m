function sim = simulate_EDL_trajectory(env, cfg)
dt = cfg.dt;
N = round(cfg.T/dt);
x = init_state(cfg, env);
sim.initialState = x.initial;
imu = init_imu(cfg);
clk.offset = 0;
sim.true.r = zeros(N,3);
sim.true.v = zeros(N,3);
sim.true.t = (1:N)'*dt;
sim.imu.a = zeros(N,3);
sim.trn.z = nan(N,3);
sim.trn.conf = zeros(N,1);
sim.trn.available = false(N,1);
sim.clock.bias = zeros(N,1);

for k = 1:N
    x = step_dynamics(x, env, cfg, dt);
    [imu, a] = sample_imu(imu, x, env, cfg, dt);
    [z, conf, available] = trn_measurement(x, env, cfg);
    clk = propagate_clock(clk, dt, cfg);

    sim.true.r(k,:) = x.r';
    sim.true.v(k,:) = x.v';
    sim.imu.a(k,:) = a';
    sim.trn.z(k,:) = z';
    sim.trn.conf(k) = conf;
    sim.trn.available(k) = available;
    sim.clock.bias(k) = clk.offset;
end
end
