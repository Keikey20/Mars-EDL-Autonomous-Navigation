function est = ekf_enhanced(sim, cfg)
% Enhanced EKF: estimates accelerometer bias and weights TRN by confidence.
dt = cfg.dt; N = size(sim.true.r,1);
xhat = [sim.initialState + [cfg.filter.initialPositionStd*randn(3,1); ...
                            cfg.filter.initialVelocityStd*randn(3,1)]; zeros(3,1)];
P = diag([repmat(cfg.filter.initialPositionStd^2,1,3), ...
          repmat(cfg.filter.initialVelocityStd^2,1,3), ...
          repmat(cfg.filter.initialBiasStd^2,1,3)]);
Q = diag([1e-4*ones(1,3), 0.02^2*ones(1,3), ...
          cfg.imu.biasWalkStd^2*ones(1,3)]);
est.xhat = zeros(N,9); est.Ptrace = zeros(N,1);

for k = 1:N
    r = xhat(1:3); v = xhat(4:6); b = xhat(7:9);
    g = gravity(r,cfg);
    a = sim.imu.a(k,:)' - b;
    xhat = [r + dt*v + 0.5*dt^2*(g+a); v + dt*(g+a); b];

    F = state_jacobian(r,dt,cfg);
    F(1:3,7:9) = -0.5*dt^2*eye(3);
    F(4:6,7:9) = -dt*eye(3);
    P = F*P*F' + Q;

    if sim.trn.available(k)
        H = [eye(3), zeros(3,6)];
        z = sim.trn.z(k,:)';
        c = max(sim.trn.conf(k),0.05);
        sigma = min(cfg.filter.enhancedMaxTrnStd, ...
                    cfg.filter.enhancedMinTrnStd/c);
        R = sigma^2*eye(3);
        K = P*H'/(H*P*H' + R);
        xhat = xhat + K*(z-H*xhat);
        I_KH = eye(9)-K*H;
        P = I_KH*P*I_KH' + K*R*K';
    end
    est.xhat(k,:) = xhat';
    est.Ptrace(k) = trace(P);
end
end

function g = gravity(r,cfg)
rn = max(norm(r),cfg.mars.radius);
g = -cfg.mars.mu/(rn^3)*r;
end

function F = state_jacobian(r,dt,cfg)
rn = max(norm(r),cfg.mars.radius);
G = -cfg.mars.mu*(eye(3)/rn^3 - 3*(r*r')/rn^5);
F = [eye(3)+0.5*dt^2*G, dt*eye(3), zeros(3); ...
     dt*G, eye(3), zeros(3); ...
     zeros(3), zeros(3), eye(3)];
end
