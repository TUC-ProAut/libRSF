function [GMM] = generateGMM(NumDim, NumComp, MeanRange, StdDevRange, StdDevFactorRange, WeightRange, IsSym, IsUnimodal, Seed)
%GENERATEGMM Summary of this function goes here
%   Detailed explanation goes here

% optinal seed reproducibility
if nargin == 9
    rng(Seed);
end

%% create model
GMM.IsSym = IsSym;
GMM.IsUnimodal = IsUnimodal;
GMM.IsValid = false;
GMM.NumDim = NumDim;
GMM.NumComp = NumComp;

% iterate until the model fullfil the requirement (in case of unimodality)
while GMM.IsValid == false
    GMM.Seed = rng;
    for nComp = 1:NumComp
        %% set mean
        % first component is always zero
        if nComp == 1 || IsSym == true
            GMM.Mean(nComp,:) = zeros(1,NumDim);
        else
            % other components depend on symmetry
            for nDim = 1:NumDim
                GMM.Mean(nComp,nDim) = sampleOutOfRange(MeanRange);
            end
        end
        
        %% set covariance
        GMM.Cov(:,:,nComp) = zeros(NumDim,NumDim);
        % sample diagonal
        if nComp == 1
            % sample StdDev direct
            for nDim = 1:NumDim
                GMM.Cov(nDim,nDim,nComp) = sampleOutOfRange(StdDevRange)^2;
            end
        else
            % sample StdDev factor
            for nDim = 1:NumDim
                Factor = sampleOutOfRange(StdDevFactorRange)^2;
                GMM.Cov(nDim,nDim,nComp) = GMM.Cov(nDim,nDim,1) * Factor^(nComp-1);
            end
        end
        
        
        %% set weight
        if NumComp == 2 && nComp == 2
            GMM.Weight(nComp) = 1 - GMM.Weight(1);
        else
            GMM.Weight(nComp) = sampleOutOfRange(WeightRange);
        end
    end
    % normalize weight
    GMM.Weight = GMM.Weight / sum(GMM.Weight);
    
    %% check model
    if IsSym == false && IsUnimodal
        
        % estimates modes with multiple resolutions and crops
        Samples = [1e2 1e3 1e4 1e5];
        Boarder = [0.05 0.1 0.15 0.2 0.25 0.3];
        NumModes = zeros(numel(Samples),numel(Boarder));
        for nSample = 1:numel(Samples)
            for nBoarder = 1:numel(Boarder)
                NumModes(nSample, nBoarder) = estimateModesGMM(GMM, Samples(nSample), Boarder(nBoarder));
            end
        end
        NumModesMax = max(NumModes,[],'all');
  
        if NumModesMax == 1
            % model is unimodal, we can use it
            GMM.IsValid = true;
        else
            % model is multimodal
            GMM.IsValid = false;
        end
    else
        % nothing to check, we are done here
        GMM.IsValid = true;
    end
    
    %% find gloabal optimum
    if GMM.IsValid
        GMM.GlobalMax = -findMaxGMM(GMM);
    end    
end
end

function [Sample] = sampleOutOfRange(Range)
Sample = (Range(2)-Range(1))*rand + Range(1);
end
