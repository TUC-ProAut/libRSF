clc
clear
close all

%% config script

% plot options
Config.SavePlot = true;
Config.DoCombinedPlot = true;
Config.DoIndividualPlot = false;

% select error model
Config.ErrorModels = {};
% Config.ErrorModels{end+1} = 'Gaussian'; % a simple Gaussian model
% Config.ErrorModels{end+1} = 'DCS';      % Dynamic Covariance Scaling
% Config.ErrorModels{end+1} = 'cDCE';     % closed-form Dynamic Covariance Estimation

Config.ErrorModels{end+1} = 'MaxMix';   % approximated GMM
Config.ErrorModels{end+1} = 'SumMix';   % exact, but badly converging GMM
Config.ErrorModels{end+1} = 'MaxSumMix'; % proposed model

% Config.ErrorModels{end+1} = 'SumMixSpecial'; % Sepcial version of Sum-Mixture with the normalization of Max-Sum Mixture

% configure GMM generaion
Config.NumModel = 1000;
Config.MeanRange = [-2 2];
Config.StdDevRange = [0.1 1];
Config.StdDevFactorRange = [2 10];
Config.WeightRange = [0.2 0.8];

% configure evaluation
Config.NumPoints = 100;
Config.RangePoints = 8;

CombinedMetric = {};

%% Case 1 (1D Sym)
Config.NumDim = 1;
Config.SymmetricGMM = true;
CombinedMetric{end+1} = MonteCarloSingle(Config);

%% Case 2 (1D Asym)
Config.NumDim = 1;
Config.SymmetricGMM = false;
CombinedMetric{end+1} = MonteCarloSingle(Config);

%% Case 3 (2D Sym)
Config.NumDim = 2;
Config.SymmetricGMM = true;
CombinedMetric{end+1} = MonteCarloSingle(Config);

%% Case 4 (2D Asym)
Config.NumDim = 2;
Config.SymmetricGMM = false;
CombinedMetric{end+1} = MonteCarloSingle(Config);

%% Case 5 (3D Sym)
Config.NumDim = 3;
Config.SymmetricGMM = true;
CombinedMetric{end+1} = MonteCarloSingle(Config);

%% Case 6 (3D Asym)
Config.NumDim = 3;
Config.SymmetricGMM = false;
CombinedMetric{end+1} = MonteCarloSingle(Config);

%% print Results
for n = 1:numel(CombinedMetric)
    Summary{n} = printCombinedMetric(CombinedMetric{n});
end

%% save summary table
ResultPath = fileparts(fileparts(fileparts(mfilename('fullpath'))));
ResultPath = [ResultPath '/Results/Estimation/Robust_Models/'];
if isfolder(ResultPath) == false
    mkdir(ResultPath)
end
ResultFilename = [ResultPath num2str(Config.NumModel) '_' num2str(Config.NumPoints) '_Summary.mat'];
save(ResultFilename, 'Config', 'Summary', 'CombinedMetric');