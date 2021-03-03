function [ WhiteError, Metric ] = evaluateCovariance3D( CovMat, Error, Lable, Metric)
%EVALUATECOVARIANCE2D Summary of this function goes here
%   Detailed explanation goes here

WhiteError = zeros(size(Error));

StdNormal= makedist('Normal','mu',0,'sigma',1);

%% whiten error

for nLable = 1:length(Lable)
    for nEle = 1:length(Error)
        W = (reshape(squeeze(CovMat(nLable,nEle,:)),[3, 3]))^(-1/2);
        
        WhiteError(nLable,nEle,:) = W * squeeze(Error(nLable,nEle,:));
    end
    
    %remove NaN
    Temp = squeeze(WhiteError(nLable,:,:));
    NotNaN = ~any(isnan(Temp),2);
    
    ErrorX = WhiteError(nLable,NotNaN,1);
    ErrorY = WhiteError(nLable,NotNaN,2);
    ErrorZ = WhiteError(nLable,NotNaN,3);   
    
    % normalized standard devivation
    Metric(nLable).StdErrX = std(ErrorX);
    Metric(nLable).StdErrY = std(ErrorY);
    Metric(nLable).StdErrZ = std(ErrorZ);
    
    % 2D normal distribution
    %MVGD{nLable} = fitgmdist([ErrorX; ErrorY]', 1);
    
    ChiSquare_med{nLable} = (ErrorX - median(ErrorX)).^2 + (ErrorY - median(ErrorY)).^2 + (ErrorZ - median(ErrorZ)).^2;
    
    ChiSquare{nLable}  = ErrorX.^2 + ErrorY.^2 + ErrorZ.^2;
    
    % calc goodness of fit
    %[ECDFChi,x] = ecdf(ChiSquare(nLable,:));
    %CDFChi = chi2cdf(x,3);
    Metric(nLable).A_NEES = mean(ChiSquare{nLable})/3;
end

Font.Size = 12;
Line.Width = 2;
Line.Color = [[0.5 0.0 0.0]; [0.0 0.5 0.0]; [0.0 0.0 0.5];...
              [0.8 0.0 0.0]; [0.0 0.8 0.0]; [0.0 0.0 0.8];...
              [0.0 0.5 0.5]; [0.5 0.0 0.5]; [0.5 0.5 0.0];...
              [0.0 0.8 0.8]; [0.8 0.0 0.8]; [0.8 0.8 0.0]];
Line.Marker.Type = {'none'; 'v'; '+'; 'x'; 's'};
Line.Marker.Size = 14;

Axis.XLable = 'X [m]';
Axis.YLable = 'Y [m]';
Axis.Width = 2;

DimNames = {'X', 'Y', 'Z'};
Plot.Size = [1600 800];

%% plot histograms
Range = 20;
PlotVector = -10:0.01:Range;
NormalDist = 1/ sqrt(2*pi) * exp(-(PlotVector.^2)./2);

% normal distribution
figure
hold on
for nLable = 1:length(Lable)
    subplot(ceil(length(Lable)/2), 2, nLable);

    hold on
    
    for nDim=1:3
        h = histogram(WhiteError(nLable,:,nDim));
        h.Normalization = 'pdf';
        h.LineStyle = 'none';
        h.BinWidth = 0.5;
        h.FaceColor = Line.Color(nDim,:);
        h.DisplayName = [Lable{nLable} ' ' DimNames{nDim}];
    end
    
    xlim([min(PlotVector) max(PlotVector)]);
    ylim([0 0.5]);

    set(gca,'FontSize',Font.Size);
    set(gca,'LineWidth',Axis.Width);
    set(gca,'layer','top') ;
    set(gcf, 'Color', 'w');
    
    p = plot(PlotVector, NormalDist, 'LineWidth',Line.Width,'Color', [0 0 0]);
    p.DisplayName = 'Normal Distribution';
    
    legend show;
    
    hold off
end
hold off
set(gcf, 'Position', [0 0 Plot.Size]);
%export_fig('-painters','-m2','Covariance.png')

% chi2 distrubution
PlotVectorChi2 = 0:0.01:60;
figure
hold on

Chi2Data = normrnd(0,1, [1 10000]).^2 + normrnd(0,1, [1 10000]).^2 + normrnd(0,1, [1 10000]).^2;
for nLable = 1:length(Lable)
    subplot(ceil(length(Lable)/2), 2, nLable);

    hold on
    
    h = histogram(ChiSquare{nLable});
    h.Normalization = 'pdf';
    h.LineStyle = 'none';
    h.BinWidth = 0.5;
    h.FaceColor = Line.Color(1,:);
    h.DisplayName = [Lable{nLable} ' Squared Sum'];
    
%     hchi2 = histogram(Chi2Data);
%     hchi2.Normalization = 'pdf';
%     hchi2.LineStyle = 'none';
%     hchi2.BinWidth = 1;
%     hchi2.FaceColor = Line.Color(2,:);
%     hchi2.DisplayName = [Lable{nLable} 'Chi Squared Sum'];

    
    xlim([min(PlotVectorChi2) max(PlotVectorChi2)]);
    ylim([0 0.25]);

    set(gca,'FontSize',Font.Size);
    set(gca,'LineWidth',Axis.Width);
    set(gca,'layer','top') ;
    set(gcf, 'Color', 'w');
    
    p = plot(PlotVectorChi2, chi2pdf(PlotVectorChi2,3), 'LineWidth',Line.Width,'Color', [0 0 0]);
    p.DisplayName = 'Chi-Squared Distribution';
    
    legend show;
    
    hold off
end
hold off
set(gcf, 'Position', [0 0 Plot.Size]);
%export_fig('-painters','-m2','CovarianceChi.png')

end

