function [RPE, TranslationError, RotationError] = calculateRelativeError(Estimate, GT, Distance, Dim)
%CALCULATERPE Summary of this function goes here
%   Detailed explanation goes here

% check input
SizeInput = size(Estimate);

% nomalize rotation
if (SizeInput(2) > SizeInput(1))...
        && ~((SizeInput(1) == 3) && (Dim == 2))...
        && ~((SizeInput(1) == 7) && (Dim == 3))
    
    Estimate = Estimate';
    GT = GT';
    
    Transpose = true;
    
    SizeInput = size(Estimate);
    
    warning("Have to transpose inputs! Size is: " + num2str(Size));
else
    Transpose = false;
end

% save dims
Length = SizeInput(1);
DOF = SizeInput(2);

% convert to affine trasformation
if Dim == 2
    
    if DOF == 3 % 2D, with rotation
        AffineEstimate = geometry.affine2D(Estimate(:,1:2), Estimate(:,3));
        AffineGT = geometry.affine2D(GT(:,1:2), GT(:,3));
    else
        error("Input dim is wrong: " + num2str(DOF));
    end
    
elseif Dim == 3
    
    if DOF == 7
        AffineEstimate = geometry.affine3D(Estimate(:,1:3), Estimate(:,4:7));
        AffineGT = geometry.affine3D(GT(:,1:3), GT(:,4:7));
    else
        error("Input dim is wrong: " + num3str(DOF));
    end
    
else
    error("Dimension not available: " + num3str(Dim));
end

% evaluate relative error over "Distance"
for n = (Length-Distance):-1:1
    AffineError(n,:,:) = (squeeze(AffineGT(n,:,:))^-1 * squeeze(AffineGT(n+Distance,:,:)))^-1 ...
                         * ...
                         (squeeze(AffineEstimate(n,:,:))^-1 * squeeze(AffineEstimate(n+Distance,:,:)));
end

% isolate translation
TranslationError = vecnorm(squeeze(AffineError(:,1:end-1,end)), 2, 2);

% apply RMSE
RPE = evaluation.calculateRMSE(TranslationError);

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

