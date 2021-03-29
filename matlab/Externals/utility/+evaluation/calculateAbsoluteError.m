function [ATE, TranslationError, RotationError] = calculateAbsoluteError(Estimate, GT, Dim)
%COMPUTEATE Summary of this function goes here
%   Detailed explanation goes here

% check input
SizeInput = size(Estimate);

% nomalize rotation
if (SizeInput(2) > SizeInput(1))...
        && ~(SizeInput(1) == 3)...
        && ~((SizeInput(1) == 2) && (Dim == 2))...
        && ~((SizeInput(1) == 7) && (Dim == 3))
    
    Estimate = Estimate';
    GT = GT';
    
    Transpose = true;
    
    SizeInput = size(Estimate);
    
    warning("Have to transpose inputs! Size is: " + num2str(SizeInput));
else
    Transpose = false;
end

% save dims
Length = SizeInput(1);
DOF = SizeInput(2);

% convert to affine trasformation
if Dim == 2
    if DOF == 2 % 2D, without rotation
        AffineEstimate = geometry.affine2D(Estimate(:,1:2), zeros(Length,1));
        AffineGT = geometry.affine2D(GT(:,1:2), zeros(Length,1));
    elseif DOF == 3 % 2D, with rotation
        AffineEstimate = geometry.affine2D(Estimate(:,1:2), Estimate(:,3));
        AffineGT = geometry.affine2D(GT(:,1:2), GT(:,3));
    else
        error("Input dim is wrong: " + num2str(DOF));
    end
    
elseif Dim == 3
    
    if DOF == 3
        AffineEstimate = geometry.affine3D(Estimate(:,1:3), [ones(Length,1) zeros(Length,3)]);
        AffineGT = geometry.affine3D(GT(:,1:3), [ones(Length,1) zeros(Length,3)]);
    elseif DOF == 7
        AffineEstimate = geometry.affine3D(Estimate(:,1:3), Estimate(:,4:7));
        AffineGT = geometry.affine3D(GT(:,1:3), GT(:,4:7));
    else
        error("Input dim is wrong: " + num2str(DOF));
    end
    
else
    error("Dimension not available: " + num2str(Dim));
end

% evaluate Error
for n = Length:-1:1
    AffineError(n,:,:) = squeeze(AffineGT(n,:,:))^-1 * squeeze(AffineEstimate(n,:,:));
end

% isolate translation
TranslationError = vecnorm( squeeze(AffineError(:,1:end-1,end)),2,2);

% apply RMSE
ATE = evaluation.calculateRMSE(TranslationError);

% isolate rotation
if Dim == 2
    RotationError = atan2(AffineError(:,2,1), AffineError(:,1,1));
elseif Dim == 3
    AngleAxisError = rotm2axang(permute(AffineError(:,1:Dim,1:Dim), [2,3,1]));
    RotationError = AngleAxisError(:,4);
end

% transpose back if required
if Transpose == true
    TranslationError = TranslationError';
    RotationError = RotationError';
end

end

