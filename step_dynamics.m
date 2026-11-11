function x = step_dynamics(x, env, cfg, dt)
% RK4 integration of point-mass Mars gravity plus exponential-atmosphere drag.
y0 = [x.r; x.v];
f = @(y) translational_rhs(y, env, cfg);
k1 = f(y0);
k2 = f(y0 + 0.5*dt*k1);
k3 = f(y0 + 0.5*dt*k2);
k4 = f(y0 + dt*k3);
y = y0 + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
x.r = y(1:3);
x.v = y(4:6);
x.t = x.t + dt;
end

function dy = translational_rhs(y, env, cfg)
r = y(1:3); v = y(4:6);
rn = max(norm(r), cfg.mars.radius);
alt = max(rn - cfg.mars.radius, 0);
rho = cfg.atm.rho0 * exp(-alt/cfg.atm.scaleHeight) * env.rhoScale;
g = -cfg.mars.mu/(rn^3) * r;
aDrag = -0.5*rho*cfg.vehicle.CdA_over_m*norm(v)*v;
dy = [v; g + aDrag];
end
