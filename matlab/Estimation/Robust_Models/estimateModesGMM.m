function [NumberMinima] = estimateModesGMM(GMM, NumSamples, Boarder)
%ESTIMATEMODESGMM Summary of this function goes here
%   Detailed explanation goes here

if nargin < 2
    NumSamples = 1e5;
    Boarder = 0.2;
elseif nargin < 3
    Boarder = 0.2;
end

%% create sampling grid
% determine dimensionality
NumDim = size(GMM.Mean, 2);

% we can use the mean to get the search area
RangeMax = max(GMM.Mean, [], 'all') + Boarder;
RangeMin = min(GMM.Mean, [], 'all') - Boarder;

% number of smaple points
NumSamplesPerDim = floor((NumSamples)^(1/NumDim));

% linear grid
Samples1D = linspace(RangeMin, RangeMax, NumSamplesPerDim);

%% create distribution object grid
GMMDist = gmdistribution(GMM.Mean, GMM.Cov, GMM.Weight);

%% search for modes
switch NumDim
    case 1
        Like =  pdf(GMMDist,Samples1D');
        PeaksAll =  islocalmax(log(Like), 'FlatSelection', 'all');
    case 2
        % create grid
        [X, Y] = meshgrid(Samples1D, Samples1D);
        Samples = [X(:), Y(:)];
        
        % sample
        Like = pdf(GMMDist, Samples);
        Like = reshape(Like, [NumSamplesPerDim NumSamplesPerDim]);

        % search over both dimensions
        PeaksAll = imregionalmax(log(Like), 4);
        
    otherwise
        error(['No mode estimation for ' num2str(NumDim) 'D implemented!']);
end

%% combine peaks
NumberMinima = sum(PeaksAll, 'all');
end