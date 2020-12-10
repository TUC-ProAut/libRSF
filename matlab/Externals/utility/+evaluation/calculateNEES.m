function [NEES] = calculateNEES(Error, Cov)
%CALCULATENEES Summary of this function goes here
%   Detailed explanation goes here
    N = size(Error,1);

    for n = N:-1:1
        NEES(n) = Error(n,:) * pinv(squeeze(Cov(n,:,:))) * Error(n,:)';
    end
end

