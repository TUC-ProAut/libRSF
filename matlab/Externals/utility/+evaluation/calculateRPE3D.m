function [RPE, TranslationalError] = calculateRPE3D(Estimate, GT)
%CALCULATERPE3D Summary of this function goes here
%   Detailed explanation goes here

    % relative movement VIO (global)
    DiffEst(:,1) = diff(Estimate(:,1));
    DiffEst(:,2) = diff(Estimate(:,2)); 
    DiffEst(:,3) = diff(Estimate(:,3));
    
    % relative movement GT (global)
    DiffGT(:,1) = diff(GT(:,1));
    DiffGT(:,2) = diff(GT(:,2)); 
    DiffGT(:,3) = diff(GT(:,3));
    
    % convert to quaternion
    QuatEst = quaternion(Estimate(:,4), Estimate(:,5), Estimate(:,6), Estimate(:,7));
    QuatGT = quaternion(GT(:,4), GT(:,5), GT(:,6), GT(:,7));

    % rotate both in a local frame
    for n = 1:numel(DiffEst(:,1))
        DiffEstLocal(n,:) = rotatepoint(QuatEst(n)^-1, DiffEst(n,:));
        DiffGTLocal(n,:) = rotatepoint(QuatGT(n)^-1, DiffGT(n,:));
    end
    
    % substract
    TranslationalError = vecnorm(DiffEstLocal - DiffGTLocal, 2, 2);
    
    % substract
    RPE = evaluation.calculateRMSE(TranslationalError);
    
end

