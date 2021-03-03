function [CombinedMetric] = MonteCarloSingle(Config)
%MONTECARLOSINGLE Summary of this function goes here
%   Detailed explanation goes here

%% create repeatable random numbers
rng('default')
Seeds = randi(2^32, Config.NumModel,1);

%% create models
parfor n = 1:Config.NumModel
    Models(n) = generateGMM(Config.NumDim, 2, Config.MeanRange, Config.StdDevRange, Config.StdDevFactorRange, Config.WeightRange, Config.SymmetricGMM, true, Seeds(n));
end

%% comparison
switch Config.NumDim
    case 1
        for n = Config.NumModel:-1:1
            [~, GMM{n}, Result{n}, Metric{n}] = CompareRobustModels1D(Config.ErrorModels, Config.RangePoints, Config.NumPoints, Models(n), false, false);
        end
    case 2
        for n = Config.NumModel:-1:1
            [~, GMM{n}, Result{n}, Metric{n}] = CompareRobustModels2D(Config.ErrorModels, Config.RangePoints, floor(sqrt(Config.NumPoints)), Models(n), false, false);
        end
    otherwise
        error(['No comparison for ' num2str(GMM.NumDim) 'D implemented!']);
end

%% combined evaluation
CombinedMetric = CombineRobustMetric(Metric);

%% set ID of this run
if Config.SymmetricGMM == true
    ID = [num2str(Config.NumModel) 'M_' num2str(Config.NumDim) 'D_' 'Sym_' ];
else
    ID = [num2str(Config.NumModel) 'M_' num2str(Config.NumDim) 'D_' 'Asym_' ];
end

%% save results
ResultPath = fileparts(fileparts(fileparts(mfilename('fullpath'))));
ResultPath = [ResultPath '/Results/Estimation/Robust_Models/'];
if isfolder(ResultPath) == false
    mkdir(ResultPath)
end
ResultFilename = [ResultPath ID 'Result.mat'];
save(ResultFilename, 'Config', 'Metric', 'Result', 'GMM', 'CombinedMetric', 'ID', 'Seeds');

%% combined plot
if Config.DoCombinedPlot == true
    PlotRobustCombined(CombinedMetric, ID, Config.SavePlot);
end

%% plot model-wise
if Config.DoIndividualPlot == true
    switch Config.NumDim
        case 1
            for n = Config.NumModel:-1:1
                PlotRobust1D(Config.ErrorModels, GMM{n}, Result{n}, Metric{n}, Config.SavePlot);
            end
        case 2
            for n = Config.NumModel:-1:1
                PlotRobust2D(Config.ErrorModels, GMM{n}, Result{n}, Metric{n}, Config.SavePlot);
            end
        otherwise
            error(['No comparison for ' num2str(GMM.NumDim) 'D implemented!']);
    end
end

end

