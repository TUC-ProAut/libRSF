function [Metric] = eval2D(Estimate, GT, varargin)
%EVAL2D Summary of this function goes here

%% check input properties
% check number of variables
narginchk(2,10);

% set default values
DefaultLable = 'Default';
DefaultCov = [];
DefaultOptimizer = [];
DefaultRuntime = 0;

% parse parameters
Parser = inputParser;
addOptional(Parser,'Cov',DefaultCov,@isnumeric);
addOptional(Parser,'Lable',DefaultLable);
addOptional(Parser,'SolverSummary',DefaultOptimizer,@isstruct);
addOptional(Parser,'Runtime',DefaultRuntime,@isnumeric);
parse(Parser,varargin{:});

% copy parsed results
Cov = Parser.Results.Cov;
Lable = Parser.Results.Lable;
SolverSummary = Parser.Results.SolverSummary;
Runtime = Parser.Results.Runtime;

% check parsing results
if isempty(Cov)
    HasCov = false;
else
    HasCov = true;
end
if isempty(SolverSummary)
    HasSolver = false;
else
    HasSolver = true;
end

% check degree of freedom
StructuralDOF = size(Estimate,2);
if HasCov
    NumericalDOF = rank(squeeze(Cov(end,:,:)));
    if StructuralDOF ~= NumericalDOF
        warning(['Uncertainty represents only a lower numerical DOF of: ' num2str(NumericalDOF)]);
    end    
end

if StructuralDOF == 2
    HasRotation = false;
elseif StructuralDOF == 3
    HasRotation = true;
else
    error('Wrong dimensionality of input data!');
end

% length of trajectorie
N = size(Estimate,1);

%% store the input data inside the metric for transparency
Metric.Lable = Lable;
Metric.Estimate = Estimate;
Metric.GT = GT;
Metric.Cov = Cov;
Metric.SolverSummary = SolverSummary;

%% error metrics

% ATE
RawError = Estimate(:,1:2) - GT(:,1:2);
Metric.ATE = vecnorm(RawError(:,1:2),2,2);
Metric.ATE_RMSE = evaluation.calculateRMSE(Metric.ATE);
Metric.ATE_Mean = mean(Metric.ATE);
Metric.ATE_Max = max(Metric.ATE);

% RPE
if HasRotation
    % rotational error
    RawError(:,3) = wrapToPi(Estimate(:,3) - GT(:,3));
    Metric.RotationalError = rad2deg(RawError(:,3));
    Metric.RotationalError_RMSE = evaluation.calculateRMSE(Metric.RotationalError);
    
    % calculate RPE
    [Metric.RPE, Metric.RelativeError] = evaluation.calculateRPE2D(Estimate, GT);
end

%% credebility metrics
if HasCov
    
    % NEES
    Metric.NEES = evaluation.calculateNEES(RawError, Cov);
    
    % average NEES
    Metric.ANEES = sum(Metric.NEES) / (NumericalDOF*N);

    % test for chi2 boundaries    
    Metric.ANEES_UpperBound = chi2inv(0.95, N*NumericalDOF)/(N*NumericalDOF);
    Metric.ANEES_LowerBound = chi2inv(0.05, N*NumericalDOF)/(N*NumericalDOF);
    Metric.ANEES_Passed = (Metric.ANEES < Metric.ANEES_UpperBound  && Metric.ANEES > Metric.ANEES_LowerBound);

    % NCI
    Metric.NCI = evaluation.calculateNCI(RawError, Cov);
    
    Metric.NCI_GeoMean = geomean(abs(Metric.NCI(Metric.NCI ~= 0)),'omitnan');
end

%% runtime metric
Metric.Runtime = Runtime;
if HasSolver
    Metric.DurationTotal = SolverSummary.DurationTotal;
    
    Metric.DurationMarginal = SolverSummary.DurationMarginal;
    
    Metric.DurationAdaptive = SolverSummary.DurationAdaptive;
    Metric.IterationAdaptive = SolverSummary.IterationAdaptive;
    
    Metric.DurationSolver = SolverSummary.DurationSolver;
    Metric.IterationSolver = SolverSummary.IterationSolver;
    
    Metric.Duration_Mean = mean(Metric.DurationSolver);
end

end

