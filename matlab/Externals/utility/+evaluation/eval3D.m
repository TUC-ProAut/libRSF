function [Metric] = eval3D(Estimate, GT, varargin)
%EVAL3D Summary of this function goes here
%   Detailed explanation goes here

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
    if StructuralDOF ~= NumericalDOF % quaternion conv as alway one DOF less!
        warning(['Uncertainty represents only a lower numerical DOF of: ' num2str(NumericalDOF)]);
    end    
end

if StructuralDOF == 3
    HasRotation = false;
elseif StructuralDOF == 7
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
[Metric.ATE, Metric.TranslationalError, Metric.RotationError] = evaluation.calculateAbsoluteError(Estimate, GT, 3);
Metric.TranslationalErrorMax = max(Metric.TranslationalError);

% RPE
if HasRotation
    % rotational error
    %TODO

    
    % relative error
    [Metric.RPE, Metric.TranslationRel, Metric.RotationRel] = evaluation.calculateRelativeError(Estimate, GT, 1, 3);
end

%% credebility metrics
if HasCov
      % calculate translation error
      RawError = Estimate(:,1:3) - GT(:,1:3);

%     % NEES
%     Metric.NEES = evaluation.calculateNEES(RawError, Cov);
      Metric.NEES_Translation = evaluation.calculateNEES(RawError(:,1:3), Cov(:,1:3,1:3));
      
      Metric.ANEES = sum(Metric.NEES_Translation, 'omitnan') / (3*N);
%     
%     % average NEES
%     Metric.ANEES = sum(Metric.NEES, 'omitnan') / (NumericalDOF*N);
% 
%     % test for chi2 boundaries    
%     Metric.ANEES_UpperBound = chi2inv(0.95, N*NumericalDOF)/(N*NumericalDOF);
%     Metric.ANEES_LowerBound = chi2inv(0.05, N*NumericalDOF)/(N*NumericalDOF);
%     Metric.ANEES_Passed = (Metric.ANEES < Metric.ANEES_UpperBound  && Metric.ANEES > Metric.ANEES_LowerBound);
% 
%     % NCI
%     Metric.NCI = evaluation.calculateNCI(RawError, Cov);
%     
%     Metric.NCI_GeoMean = geomean(abs(Metric.NCI(Metric.NCI ~= 0)),'omitnan');
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
    
    Metric.Duration_Mean = mean(Metric.DurationTotal);
    Metric.Duration_Mean_ms = Metric.Duration_Mean*1000;
end

end

