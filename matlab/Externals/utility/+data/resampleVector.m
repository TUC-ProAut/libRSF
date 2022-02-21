function [Y2] = resampleVector(X1, Y1, X2)
%RESAMPLEVECTOR Summary of this function goes here

    % resampling
    Y2 = interp1(X1, Y1, X2, 'linear', nan);

    % fix values that are broken by extrapolation
    if any(ismissing(Y2))
        Y2 = fillmissing(Y2,'nearest',1);
    end
end

