function [RPE, RelativeError] = calculateRPE2D(Estimate, GT)
%CALCULATERPE2D Summary of this function goes here
%   Detailed explanation goes here

% relative motion
DiffTransEst = diff(Estimate(:,1:2),1);
DiffTransGT = diff(GT(:,1:2),1);

% rotate in local frame
for n = size(DiffTransEst, 1):-1:1
    DiffTransEstRot(n,:) = DiffTransEst(n,:) * geometry.rot2D(Estimate(n,3));
    DiffTransGTRot(n,:) = DiffTransGT(n,:) * geometry.rot2D(GT(n,3));
end

% calculate RPE
RelativeError = vecnorm(DiffTransEstRot - DiffTransGTRot,2,2);
RPE = evaluation.calculateRMSE(DiffTransEstRot - DiffTransGTRot);

end

