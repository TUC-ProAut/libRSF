function [Summary] = createSummary(Metric)
%CREATESUMMARY Summary of this function goes here
%   Detailed explanation goes here

%% get relevant informations
Algorithm = {Metric.Lable}';
ATE = [Metric.ATE_RMSE]';
ATE_Max = [Metric.ATE_Max]';

if isfield(Metric, 'ANEES')
    ANEES = [Metric.ANEES]';
else
    ANEES = [Metric.ATE_RMSE]'*0; % fill with zero
end

if isfield(Metric, 'NCI_GeoMean')
    NCI = [Metric.NCI_GeoMean]';
else
    NCI = [Metric.ATE_RMSE]'*0; % fill with zero
end

if isfield(Metric, 'RPE')
    RPE = [Metric.RPE]';
else
    RPE = [Metric.ATE_RMSE]'*0; % fill with zero
end

if isfield(Metric, 'Duration_Mean')
    Iteration_ms = [Metric.Duration_Mean]'*1000;
else
    Iteration_ms = [Metric.ATE_RMSE]'*0; % fill with zero
end

Runtime = [Metric.Runtime]';

Summary = table(Algorithm, ATE, ATE_Max, RPE, ANEES, NCI, Runtime, Iteration_ms);

end

