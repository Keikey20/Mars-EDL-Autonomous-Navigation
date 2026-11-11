function env = sample_environment(cfg)
env.rhoScale = max(0.5, 1 + cfg.atm.rhoScaleSigma*randn);
env.dustTau = max(0.05, 0.35 + 0.15*randn);
end
