function [CombinedMetric] = CombineRobustMetric(Metric)
%COMBINEROBUSTMETRIC Summary of this function goes here
%   Detailed explanation goes here
NumModel = numel(Metric);
NumError = numel(Metric{1});

for n = 1:NumModel
    for m = 1:NumError
        if n == 1
            CombinedMetric(m).Label = Metric{n}(m).Label;
            CombinedMetric(m).Error = [];
            CombinedMetric(m).Duration = [];
            CombinedMetric(m).Iterations = [];
        end
        CombinedMetric(m).Error_RMS_Model(n) = Metric{n}(m).Error_RMS;
        CombinedMetric(m).Error = [CombinedMetric(m).Error; Metric{n}(m).Error];
        CombinedMetric(m).Duration = [CombinedMetric(m).Duration; Metric{n}(m).Duration];
        CombinedMetric(m).Iterations = [CombinedMetric(m).Iterations; Metric{n}(m).Iterations];
    end
end
% process
for m = 1:NumError
    CombinedMetric(m).Error_RMS = evaluation.calculateRMSE(CombinedMetric(m).Error);
    CombinedMetric(m).Error_Max = max(CombinedMetric(m).Error);
    CombinedMetric(m).Iterations_Mean = mean(CombinedMetric(m).Iterations);
    CombinedMetric(m).Duration_Mean = mean(CombinedMetric(m).Duration);
    CombinedMetric(m).Convegence_Rate = sum(abs(CombinedMetric(m).Error) < 1e-2)/ numel(CombinedMetric(m).Error);
end
end

