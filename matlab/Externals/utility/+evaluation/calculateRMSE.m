function [RMSE] = calculateRMSE(Error)
%CALCULATERMSE Summary of this function goes here
%   Detailed explanation goes here
    RMSE = sqrt(mean(vecnorm(Error,2,2).^2, 'omitnan'));
end

