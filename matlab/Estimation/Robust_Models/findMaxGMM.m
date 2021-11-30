function [GlobalMax] = findMaxGMM(GMM)
%FINDMAXGMM Summary of this function goes here
%   Detailed explanation goes here

% determine dimensionality
NumDim = size(GMM.Mean, 2);

% determine interesting window
GridMin = min(GMM.Mean, [], 'all');
GridMax = max(GMM.Mean, [], 'all');

% create sample points
Samples1D = linspace(GridMin, GridMax, 1e4^(1/NumDim));
switch NumDim
    case 1
        Samples = Samples1D';
    case 2
        [X,Y] = meshgrid(Samples1D, Samples1D);
        Samples = [X(:), Y(:)];
    otherwise
        error(['No mode estimation for ' num2str(NumDim) 'D implemented!']);
end

% create distribution object
GMMDist = gmdistribution(GMM.Mean, GMM.Cov, GMM.Weight);

% find sample minimum
[~, iMax] = max(pdf(GMMDist, Samples));

% optimize for true minimum
Opt = optimset('TolFun', 1e-6, 'TolX', 1e-6);
GlobalMax = fminsearch(@(x)-log(pdf(GMMDist, x)), Samples(iMax,:), Opt);

end

