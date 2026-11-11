
function clk = propagate_clock(clk, dt, cfg)

% Deterministic clock drift
clk.offset = clk.offset + cfg.clock.drift * dt;

% Random clock jitter
clk.offset = clk.offset + cfg.clock.jitterStd * randn;

end
