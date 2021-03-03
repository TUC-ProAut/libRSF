function [Summary] = printCombinedMetric(CombinedMetric)
%PRINTCOMBINEDMETRIC Summary of this function goes here
%   Detailed explanation goes here

% create summary table
Algorithm = {CombinedMetric.Lable}';
EstimationErrorRMS = [CombinedMetric.Error_RMS]';
%EstimationErrorMax = [CombinedMetric.Error_Max]';
Runtime_us = [CombinedMetric.Duration_Mean]'*1e6;
Iterations = [CombinedMetric.Iterations_Mean]';
Convegence_Rate = [CombinedMetric.Convegence_Rate]'*100;
Summary = table(Algorithm, EstimationErrorRMS, Convegence_Rate, Iterations, Runtime_us);

% print summary
disp(Summary);

end

