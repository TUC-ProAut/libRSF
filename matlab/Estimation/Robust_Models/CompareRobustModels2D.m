function [ErrorModels, GMM, Result, Metric] = CompareRobustModels2D(ErrorModels, Range, Points, GMM, DoPlot, SavePlot)

%% default config
if nargin < 4
    clc
    clear
    close all
    
    DoPlot = true;
    SavePlot = true;
    
    ErrorModels = {'MaxMix','SumMix', 'MaxSumMix'};
    
    Range = 8;
    Points = round(sqrt(10000));
    
    % default model    
    GMM.Mean(1,:)= [0 0];
    GMM.Mean(2,:) = [2 1];
    GMM.Cov(:,:,1) = [1 0; 0 1].^2;
    GMM.Cov(:,:,2) = [2 0; 0 2].^2;
    GMM.Weight = [0.5, 0.5];
    
    GMM.GlobalMax = -findMaxGMM(GMM);
end

%% preprocessing
if DoPlot == true
    % construct GMM
    GMMDist = gmdistribution(GMM.Mean, GMM.Cov, GMM.Weight);

    % create samples
    Samples = linspace(-Range/2,Range/2,Points);
    Index = 1;
    for n=1:numel(Samples)
        for m=1:numel(Samples)
            GMM.Samples(Index,:) = [Samples(m) Samples(n)];
            Index = Index+1;
        end
    end
    GMM.Prob = pdf(GMMDist, -GMM.Samples);

    disp(['Global optimum at: ' num2str(GMM.GlobalMax)]);
end

%% call ceres
for n = numel(ErrorModels):-1:1
    Config.Custom = [num2str(Points) ' ' num2str(Range) ' '...
        ErrorModels{n} ' ' ...
        num2str(GMM.Mean(1,1)) ' ' num2str(GMM.Mean(1,2)) ' '...
        num2str(GMM.Mean(2,1)) ' ' num2str(GMM.Mean(2,2)) ' '...
        num2str(sqrt(GMM.Cov(1,1,1))) ' ' num2str(sqrt(GMM.Cov(1,2,1))) ' ' num2str(sqrt(GMM.Cov(2,1,1))) ' ' num2str(sqrt(GMM.Cov(2,2,1))) ' ' ...
        num2str(sqrt(GMM.Cov(1,1,2))) ' ' num2str(sqrt(GMM.Cov(1,2,2))) ' ' num2str(sqrt(GMM.Cov(2,1,2))) ' ' num2str(sqrt(GMM.Cov(2,2,2))) ' ' ...
        num2str(GMM.Weight(1)) ' ' num2str(GMM.Weight(2))];
    ResultCeres(n) = libRSF.wrapCeres([], Config, 'App_Robust_Models_2D', 'Data_2D', true, false);
end

%% assigne output
for n = numel(ErrorModels):-1:1
    Result.Raw(n) = ResultCeres(n);
    Result.Error(:,:,n) = -ResultCeres(n).Cost2.Position;
    Result.Cost(:,n) = ResultCeres(n).Cost2.Cost;
    Result.Gradient(:,:,n) = -ResultCeres(n).Cost2.Gradient;
    Result.HessianNorm(:,n) = vecnorm(ResultCeres(n).Cost2.Hessian, 2, 2);
    
    Result.PreOpt(:,1,n)= ResultCeres(n).Position2.X(ResultCeres(n).Position2.Time < 0.5);
    Result.PreOpt(:,2,n)= ResultCeres(n).Position2.Y(ResultCeres(n).Position2.Time < 0.5);
    
    Result.PostOpt(:,1,n)= ResultCeres(n).Position2.X(ResultCeres(n).Position2.Time > 0.5);
    Result.PostOpt(:,2,n)= ResultCeres(n).Position2.Y(ResultCeres(n).Position2.Time > 0.5);
    
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

%% Plot
if DoPlot
    PlotRobust2D(ErrorModels, GMM, Result, Metric, SavePlot);
end

end