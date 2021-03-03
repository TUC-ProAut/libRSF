function [ErrorModels, GMM, Result, Metric] = CompareRobustModels1D(ErrorModels, Range, Points, GMM, DoPlot, SavePlot)
%COMPAREROBUSTMODELS compare robust error models of the libRSF

%% default config
if nargin < 4
    clc
    clear
    close all
    
    DoPlot = true;
    SavePlot = true;

    ErrorModels = {'MaxMix','SumMix', 'MaxSumMix'};
    
    Range = 8;
    Points = 1000;
    
    % default model
    GMM.Mean(1:2,1) = [0 2];
    GMM.Cov(1,1,1:2) = [1 2].^2;
    GMM.Weight = [0.5 0.5];

    GMM.GlobalMax = -findMaxGMM(GMM);
end

%% calculate scaling terms for probabilities
GMM.StdDev = squeeze(GMM.Cov).^(1/2);
Scaling = zeros(numel(ErrorModels),1);
for n = 1:numel(ErrorModels)
    switch ErrorModels{n}
        case 'SumMix'
            Scaling(n) = GMM.Weight(1)/GMM.StdDev(1) + GMM.Weight(2)/GMM.StdDev(2);
            
        case 'MaxMix'
            Scaling(n) = max(GMM.Weight(1)/GMM.StdDev(1) , GMM.Weight(2)/GMM.StdDev(2));
            
        case 'MaxSumMix'
            Scaling(n) = max(GMM.Weight(1)/GMM.StdDev(1) , GMM.Weight(2)/GMM.StdDev(2)) * 2 + 10;
            
        otherwise
            Scaling(n) = 1/GMM.StdDev(1);
    end
end

%% preprocessing
if DoPlot == true
    % construct GMM
    GMMDist = gmdistribution(GMM.Mean, GMM.Cov, GMM.Weight);

    % sample the GMM
    GMM.Samples = linspace(-Range/2, Range/2, Points)';
    GMM.Prob = pdf(GMMDist, -GMM.Samples);
    GMM.NegLogLike = -log(GMM.Prob);
    GMM.Jacobian = gradient(GMM.NegLogLike, GMM.Samples);
    GMM.Hessian = gradient(GMM.Jacobian, GMM.Samples);

    disp(['Global optimum at: ' num2str(GMM.GlobalMax)]);
end

%% call ceres
for n = numel(ErrorModels):-1:1
    Config.Custom = [num2str(Points) ' ' num2str(Range) ' '...
        ErrorModels{n} ' ' ...
        num2str(GMM.Mean(1,1)) ' ' num2str(GMM.Mean(2,1)) ' '...
        num2str(sqrt(GMM.Cov(1,1,1))) ' ' num2str(sqrt(GMM.Cov(1,1,2))) ' ' ...
        num2str(GMM.Weight(1)) ' ' num2str(GMM.Weight(2))];    
    ResultCeres(n) = libRSF.wrapCeres([], Config, 'Example_Robust_Models_1D', 'Data_1D', true, false);
end

%% assigne output
for n = numel(ErrorModels):-1:1
    Result.Raw(n) = ResultCeres(n);
    Result.Error(:,n) = -ResultCeres(n).Cost1.Position;
    Result.Cost(:,n) = ResultCeres(n).Cost1.Cost;
    Result.Gradient(:,n) = -ResultCeres(n).Cost1.Gradient;
    Result.Hessian(:,n) = ResultCeres(n).Cost1.Hessian;
    
    Result.PreOpt(:,n)= ResultCeres(n).Position1.X(ResultCeres(n).Position1.Time < 0.5);
    Result.PostOpt(:,n)= ResultCeres(n).Position1.X(ResultCeres(n).Position1.Time > 0.5);
    
    Result.Duration(:,n) = ResultCeres(n).SolverSummary.DurationSolver;
    Result.Iterations(:,n) = ResultCeres(n).SolverSummary.IterationSolver;
end

%% calculate probability
for n = numel(ErrorModels):-1:1
    Result.Prob(:,n) = exp(-Result.Cost(:,n)) .* 1/sqrt(2*pi) .* Scaling(n);
    if Points > 1
        Result.Sum(n) = trapz(-Result.Error(:,n), Result.Prob(:,n));
    else
        Result.Sum(n) = 0;
    end
end

%% evaluation
Metric = [];
for n = numel(ErrorModels):-1:1
    Metric(n).Lable = ErrorModels{n};
    
    % accuracy
    if strcmp(ErrorModels{n}, 'Gaussian') || strcmp(ErrorModels{n}, 'DCS') || strcmp(ErrorModels{n}, 'cDCE')
        Metric(n).Error = Result.PostOpt(:,n);      
    else
        Metric(n).Error = Result.PostOpt(:,n) - GMM.GlobalMax;
    end
    Metric(n).Error_RMS = evaluation.calculateRMSE(Metric(n).Error);
    
    % speed
    Metric(n).Duration = Result.Duration(:,n);
    Metric(n).Duration_Mean = mean(Metric(n).Duration);
    Metric(n).Iterations = Result.Iterations(:,n);
    Metric(n).Iterations_Mean = mean(Metric(n).Iterations);
    
    % probabilistics
    Metric(n).Prob = Result.Prob(:,n);
    Metric(n).Prob_Integral = Result.Sum(n);
end
% print summary
Algorithm = {Metric.Lable}';
EstimationError = [Metric.Error_RMS]';
Runtime_ms = [Metric.Duration_Mean]'*1e3;
Iterations = [Metric.Iterations_Mean]';
Integral = [Metric.Prob_Integral]';
Summary = table(Algorithm, EstimationError, Runtime_ms, Iterations, Integral);
if DoPlot == true
    disp(Summary);
end

%% plot the results if desired
if DoPlot
    PlotRobust1D(ErrorModels, GMM, Result, Metric, SavePlot)
end

end

