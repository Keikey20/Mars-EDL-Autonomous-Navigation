function results = main_MC()
% Run from this folder: results = main_MC;
cfg = make_config();
rng(cfg.seed);
Nmc = cfg.Nmc;
errBase = zeros(Nmc,1); errEnh = zeros(Nmc,1);
firstSim = []; firstBase = []; firstEnh = [];

for j = 1:Nmc
    env = sample_environment(cfg);
    sim = simulate_EDL_trajectory(env,cfg);
    base = ekf_update(sim,cfg);
    enh = ekf_enhanced(sim,cfg);
    truth = sim.true.r(end,:)';
    errBase(j) = norm(truth-base.xhat(end,1:3)');
    errEnh(j) = norm(truth-enh.xhat(end,1:3)');
    if j == 1
        firstSim=sim; firstBase=base; firstEnh=enh;
    end
end

results.baselineErrors = errBase;
results.enhancedErrors = errEnh;
results.baselineMean = mean(errBase);
results.enhancedMean = mean(errEnh);
results.baselineStd = std(errBase);
results.enhancedStd = std(errEnh);

fprintf('Baseline EKF: mean final position error %.3f m (std %.3f m)\n', ...
    results.baselineMean,results.baselineStd);
fprintf('Enhanced EKF: mean final position error %.3f m (std %.3f m)\n', ...
    results.enhancedMean,results.enhancedStd);

outdir = fullfile(pwd,'Results');
if ~exist(outdir,'dir'), mkdir(outdir); end

fig=figure('Visible','off');
scatter(errBase,errEnh,28,'filled'); grid on; axis equal;
xlabel('Baseline final position error [m]');
ylabel('Enhanced final position error [m]');
title('Monte Carlo final position error comparison');
saveas(fig,fullfile(outdir,'monte_carlo_error_comparison.png')); close(fig);

fig=figure('Visible','off');
histogram(errBase,15); hold on; histogram(errEnh,15);
xlabel('Final position error [m]'); ylabel('Count');
legend('Baseline EKF','Enhanced EKF'); grid on;
title('Landing error distribution');
saveas(fig,fullfile(outdir,'landing_error_distribution.png')); close(fig);

t=firstSim.true.t;
fig=figure('Visible','off');
plot(t,firstSim.true.r-firstEnh.xhat(:,1:3));
xlabel('Time [s]'); ylabel('Position error [m]');
legend('x','y','z'); grid on; title('Enhanced EKF position error');
saveas(fig,fullfile(outdir,'position_error_vs_time.png')); close(fig);

fig=figure('Visible','off');
plot(t,firstEnh.Ptrace); xlabel('Time [s]'); ylabel('Covariance trace');
grid on; title('Enhanced EKF covariance trace');
saveas(fig,fullfile(outdir,'ekf_uncertainty.png')); close(fig);

fig=figure('Visible','off');
plot(t,firstSim.trn.conf); xlabel('Time [s]'); ylabel('TRN confidence');
grid on; title('TRN confidence');
saveas(fig,fullfile(outdir,'trn_confidence.png')); close(fig);

fig=figure('Visible','off');
plot(t,firstSim.clock.bias); xlabel('Time [s]'); ylabel('Clock offset [s]');
grid on; title('Onboard clock offset');
saveas(fig,fullfile(outdir,'clock_offset.png')); close(fig);

save(fullfile(outdir,'monte_carlo_results.mat'),'results','firstSim','firstBase','firstEnh','cfg');
end
