save('Mars_EDL_Final_Results.mat','results','T');

writetable(T,'Mars_EDL_Statistical_Comparison.xlsx', ...
    'WriteRowNames',true);