function [NEES] = calculateNEES(Error, Cov)
%CALCULATENEES Summary of this function goes here
%   Detailed explanation goes here
    N = size(Error,1);

    for n = N:-1:1
        Info = pinv(squeeze(Cov(n,:,:)));
        if all(eig(Info) > 0)
            NEES(n) = Error(n,:) * Info * Error(n,:)';
        else
            NEES(n) = nan;
        end
    end
end

