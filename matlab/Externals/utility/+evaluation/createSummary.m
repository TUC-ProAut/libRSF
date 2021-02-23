function [Summary] = createSummary(Metric)
%CREATESUMMARY Summary of this function goes here
%   Detailed explanation goes here

%% get relevant informations
Algorithm = {Metric.Lable}';
ATE = [Metric.ATE]';
ATE_Max = [Metric.TranslationalErrorMax]';

if isfield(Metric, 'ANEES')
    ANEES = [Metric.ANEES]';
else
    ANEES = zeros(size(ATE));
end

if isfield(Metric, 'RPE')
    RPE = [Metric.RPE]';
else
    RPE = zeros(size(ATE));
end

if isfield(Metric, 'Duration_Mean')
    Iteration_ms = [Metric.Duration_Mean]'*1000;
else
    Iteration_ms = zeros(size(ATE));
end

Runtime = [Metric.Runtime]';

Summary = table(Algorithm, ATE, ATE_Max, RPE, ANEES, Runtime, Iteration_ms);

end

