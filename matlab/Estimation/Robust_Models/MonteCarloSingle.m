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
        parfor n = 1:Config.NumModel
            [~, GMM{n}, Result{n}, Metric{n}] = CompareRobustModels1D(Config.ErrorModels, Config.RangePoints, Config.NumPoints, Models(n), false, false);
        end
    case 2
        parfor n = 1:Config.NumModel
            [~, GMM{n}, Result{n}, Metric{n}] = CompareRobustModels2D(Config.ErrorModels, Config.RangePoints, floor(sqrt(Config.NumPoints)), Models(n), false, false);
        end
    case 3
        parfor n = 1:Config.NumModel
            [~, GMM{n}, Result{n}, Metric{n}] = CompareRobustModels3D(Config.ErrorModels, Config.RangePoints, floor(sqrt(Config.NumPoints)), Models(n), false, false);
        end
    otherwise
        error(['No comparison for ' num2str(Config.NumDim) 'D implemented!']);
end

%% combined evaluation
CombinedMetric = CombineRobustMetric(Metric);

%% set ID of this run
BaseName = [num2str(Config.NumModel) 'M_' num2str(Config.NumPoints) 'P_' num2str(Config.NumDim) 'D_' ];

if Config.SymmetricGMM == true
    BaseName = [BaseName 'Sym'];
else
    BaseName = [BaseName 'Asym'];
end

%% save results
PlotPath = estimation.post.getPlotPath('Robust_Models');
SavePath = estimation.post.getSavePath('Robust_Models');
SaveFile = fullfile(SavePath, BaseName);
save(SaveFile, 'Config', 'Metric', 'Result', 'GMM', 'CombinedMetric', 'BaseName', 'Seeds');

%% combined plot
if Config.DoCombinedPlot == true
    PlotRobustCombined(SaveFile, PlotPath, Config.SavePlot);
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

