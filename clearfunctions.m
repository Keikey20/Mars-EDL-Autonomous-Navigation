clear functions
results = main_MC;

figure;

histogram(results.baselineErrors, 15);
hold on;
histogram(results.enhancedErrors, 15);

xlabel('Final Position Error (m)');
ylabel('Number of Trials');
title('Monte Carlo Comparison: Baseline vs Enhanced EKF');
legend('Baseline EKF','Enhanced EKF');
grid on;