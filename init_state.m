function x = init_state(cfg, env)
% Planet-fixed Cartesian frame: +x radial outward, +y downrange.
R = cfg.mars.radius;
r0 = R + cfg.entry.altitude;
gamma = deg2rad(cfg.entry.gammaDeg);
v = cfg.entry.speed;
x.r = [r0; 0; 0];
% Negative radial velocity for a descending entry; tangential component +y.
x.v = [v*sin(gamma); v*cos(gamma); 0];
x.q = [1;0;0;0]; % attitude held fixed in this simplified model
x.t = 0;
x.initial = [x.r; x.v];
end
