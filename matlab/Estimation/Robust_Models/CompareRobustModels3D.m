function [ErrorModels, GMM, Result, Metric] = CompareRobustModels3D(ErrorModels, Range, Points, GMM, DoPlot, SavePlot)

%% default config
if nargin < 4
    clc
    clear
    close all
    
    DoPlot = true;
    SavePlot = true;
    
    ErrorModels = {'MaxMix','SumMix', 'MaxSumMix'};
    
    Range = 8;
    Points = round((1000)^(1/3));
    
    % default model    
    GMM.Mean(1,:)= [0 0 0];
    GMM.Mean(2,:) = [2 1 2];
    GMM.Cov(:,:,1) = eye(3).^3;
    GMM.Cov(:,:,2) = (eye(3)*2).^3;
    GMM.Weight = [0.5, 0.5];
    
    GMM.GlobalMax = -findMaxGMM(GMM);
end

%% call ceres
for n = numel(ErrorModels):-1:1
    Config.Custom = [num2str(Points) ' ' num2str(Range) ' '...
        ErrorModels{n} ' ' ...
        num2str(GMM.Mean(1,1)) ' ' num2str(GMM.Mean(1,2)) ' ' num2str(GMM.Mean(1,3)) ' '...
        num2str(GMM.Mean(2,1)) ' ' num2str(GMM.Mean(2,2)) ' ' num2str(GMM.Mean(2,3)) ' '...
        num2str(sqrt(GMM.Cov(1,1,1))) ' ' num2str(sqrt(GMM.Cov(1,2,1))) ' ' num2str(sqrt(GMM.Cov(1,3,1))) ' ' ...
        num2str(sqrt(GMM.Cov(2,1,1))) ' ' num2str(sqrt(GMM.Cov(2,2,1))) ' ' num2str(sqrt(GMM.Cov(2,3,1))) ' ' ...
        num2str(sqrt(GMM.Cov(3,1,1))) ' ' num2str(sqrt(GMM.Cov(3,2,1))) ' ' num2str(sqrt(GMM.Cov(3,3,1))) ' ' ...
        num2str(sqrt(GMM.Cov(1,1,2))) ' ' num2str(sqrt(GMM.Cov(1,2,2))) ' ' num2str(sqrt(GMM.Cov(1,3,2))) ' ' ...
        num2str(sqrt(GMM.Cov(2,1,2))) ' ' num2str(sqrt(GMM.Cov(2,2,2))) ' ' num2str(sqrt(GMM.Cov(2,3,2))) ' ' ...
        num2str(sqrt(GMM.Cov(3,1,2))) ' ' num2str(sqrt(GMM.Cov(3,2,2))) ' ' num2str(sqrt(GMM.Cov(3,3,2))) ' ' ...
        num2str(GMM.Weight(1)) ' ' num2str(GMM.Weight(2))];
    ResultCeres(n) = libRSF.wrapCeres([], Config, 'App_Robust_Models_3D', 'Data_3D', true, false);
end

%% assigne output
for n = numel(ErrorModels):-1:1
    Result.Raw(n) = ResultCeres(n);
    Result.Error(:,:,n) = -ResultCeres(n).Cost3.Position;
    Result.Cost(:,n) = ResultCeres(n).Cost3.Cost;
    Result.Gradient(:,:,n) = -ResultCeres(n).Cost3.Gradient;
    Result.HessianNorm(:,n) = vecnorm(ResultCeres(n).Cost3.Hessian, 2, 2);
    
    Result.PreOpt(:,1,n)= ResultCeres(n).Position3.X(ResultCeres(n).Position3.Time < 0.5);
    Result.PreOpt(:,2,n)= ResultCeres(n).Position3.Y(ResultCeres(n).Position3.Time < 0.5);
    Result.PreOpt(:,3,n)= ResultCeres(n).Position3.Z(ResultCeres(n).Position3.Time < 0.5);
    
    Result.PostOpt(:,1,n)= ResultCeres(n).Position3.X(ResultCeres(n).Position3.Time > 0.5);
    Result.PostOpt(:,2,n)= ResultCeres(n).Position3.Y(ResultCeres(n).Position3.Time > 0.5);
    Result.PostOpt(:,3,n)= ResultCeres(n).Position3.Z(ResultCeres(n).Position3.Time > 0.5);
    
    Result.Duration(:,n) = ResultCeres(n).SolverSummary.DurationSolver;
    Result.Iterations(:,n) = ResultCeres(n).SolverSummary.IterationSolver;
end

%% evaluation
Metric = [];
for n = numel(ErrorModels):-1:1
    Metric(n).Lable = ErrorModels{n};
    
    % accuracy
    if strcmp(ErrorModels{n}, 'Gaussian') || strcmp(ErrorModels{n}, 'DCS') || strcmp(ErrorModels{n}, 'cDCE')
        Metric(n).Error = vecnorm(squeeze(Result.PostOpt(:,:,n)),2,2);    
    else
        Metric(n).Error = vecnorm(squeeze(Result.PostOpt(:,:,n)) - GMM.GlobalMax,2,2);
    end
    Metric(n).Error_RMS = evaluation.calculateRMSE(Metric(n).Error);
    
    % speed
    Metric(n).Duration = Result.Duration(:,n);
    Metric(n).Duration_Mean = mean(Metric(n).Duration);
    Metric(n).Iterations = Result.Iterations(:,n);
    Metric(n).Iterations_Mean = mean(Metric(n).Iterations);
end
% print summary
Algorithm = {Metric.Lable}';
EstimationError = [Metric.Error_RMS]';
Runtime_ms = [Metric.Duration_Mean]'*1e3;
Iterations = [Metric.Iterations_Mean]';
Summary = table(Algorithm, EstimationError, Runtime_ms, Iterations);
if DoPlot == true
    disp(Summary);
end

end