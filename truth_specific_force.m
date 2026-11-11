function f = truth_specific_force(x, env, cfg)
% Specific force is non-gravitational acceleration; body and inertial axes
% are treated as aligned because attitude dynamics are outside this model.
r = x.r(:); v = x.v(:);
rn = max(norm(r), cfg.mars.radius);
alt = max(rn - cfg.mars.radius, 0);
rho = cfg.atm.rho0 * exp(-alt/cfg.atm.scaleHeight) * env.rhoScale;
f = -0.5*rho*cfg.vehicle.CdA_over_m*norm(v)*v;
end
