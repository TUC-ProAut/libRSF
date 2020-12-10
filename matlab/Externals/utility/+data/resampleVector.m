function [Y2] = resampleVector(X1, Y1, X2)
%RESAMPLEVECTOR Summary of this function goes here

    % resample data Y1 according to new 'time' vector X2
    TS = timeseries(Y1, X1);
    TS = resample(TS, X2);
    Y2 = TS.data;
    
    % fix values that are broken by extrapolation
    if any(ismissing(Y2))
        Y2 = fillmissing(Y2,'nearest',1);
    end
end

