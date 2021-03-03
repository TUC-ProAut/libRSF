function [NCI] = calculateNCI(Error, Cov)
%CALCULATENCI Summary of this function goes here
%   Detailed explanation goes here
    N = size(Error,1);

    ActualNEES = evaluation.calculateNEES(Error, Cov);
    
    MSE = permute(repmat(cov(Error), 1, 1, N), [3 1 2]);
    IdealNEES = evaluation.calculateNEES(Error, MSE);
    
    
    NCI = ActualNEES ./ IdealNEES;
end

