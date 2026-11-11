
figure('Color','w');

boxplot([results.baselineErrors, results.enhancedErrors], ...
    'Labels', {'Baseline EKF','Enhanced EKF'});

ylabel('Final Position Error (m)');
title('Monte Carlo Comparison of EKF Position Estimation');
set(gca,'FontSize',12);
grid on;
box on;

exportgraphics(gcf,'EKF_Position_Error_Boxplot.png','Resolution',300);