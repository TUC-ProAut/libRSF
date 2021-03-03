function [ WhiteError, Metric ] = evaluateCovariance2D( CovMat, Error, Lable, Metric)
%EVALUATECOVARIANCE2D Summary of this function goes here
%   Detailed explanation goes here

WhiteError = zeros(size(Error));

StdNormal= makedist('Normal','mu',0,'sigma',1);

%% whiten error

for nLable = 1:length(Lable)
    for nEle = 1:length(Error)
        W = (reshape(squeeze(CovMat(nLable,nEle,:)),[2, 2]))^(-1/2);
        WhiteError(nLable,nEle,:) = real(W * squeeze(Error(nLable,nEle,:)));
    end
    
    ErrorX = WhiteError(nLable,1:end,1);
    ErrorY = WhiteError(nLable,1:end,2);
    
    % normalized standard devivation
    Metric(nLable).StdErrX = std(ErrorX, 'omitnan');
    Metric(nLable).StdErrY = std(ErrorY, 'omitnan');
    
    Metric(nLable).ANEES = mean([mean(ErrorX.^2, 'omitnan'), mean(ErrorY.^2, 'omitnan')]);
    
    % 2D normal distribution
    MVGD{nLable} = fitgmdist([ErrorX; ErrorY]', 1);
    
    % calc goodness of fit
%     [fx,x] = ecdf(ErrorX);
%     f_Gx = cdf(StdNormal,x);
%     [fy,x] = ecdf(ErrorY);
%     f_Gy = cdf(StdNormal,x);
%     Metric(nLable).GoFX = sum((fx-f_Gx).^2);
%     Metric(nLable).GoFY = sum((fy-f_Gy).^2);
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

%% plot histograms
Range = 7;
PlotVector = -Range:0.01:Range;
NormalDist = 1/ sqrt(2*pi) * exp(-(PlotVector.^2)./2);
figure
hold on
for nLable = 1:length(Lable)
    subplot(ceil(length(Lable)/3), 3, nLable);

    hold on
    h = histogram(WhiteError(nLable,:,1));
    h.Normalization = 'pdf';
    h.LineStyle = 'none';
    h.BinWidth = 0.1;
    h.FaceColor = Line.Color(nLable,:);
    
    legend(h, Lable{nLable});
    
    xlim([-Range Range]);
    ylim([0 0.6]);

    set(gca,'FontSize',Font.Size);
    set(gca,'LineWidth',Axis.Width);
    set(gca,'layer','top') ;
    set(gcf, 'Color', 'w');
    
    p = plot(PlotVector, NormalDist, 'LineWidth',Line.Width,'Color', [0 0 0]);
    %p.DisplayName = 'Normal Distribution';
    
    hold off
end
hold off

%% plot 2D distribution
% figure
% hold on
% for nLable = 1:length(Lable)
%     subplot(ceil(length(Lable)/3), 3, nLable);
% 
%     hold on
%     
%     h = scatter(WhiteError(nLable,:,1), WhiteError(nLable,:,2),'.');
%     h.MarkerEdgeColor = Line.Color(nLable,:);
%     
%     ConfidenceLevel = 0.9973;
%     Style = {'Color', Line.Color(nLable,:).*0.5,'LineWidth',Line.Width, 'LineStyle', '--'};
%     StyleGT = {'Color', [0 0 0],'LineWidth',Line.Width, 'LineStyle', '-'};
%     f = plot_errorEllipse(MVGD{nLable}.Sigma, MVGD{nLable}.mu, 'conf', ConfidenceLevel, 'style', Style);
%     GT = plot_errorEllipse([1 0; 0 1], [0 0], 'conf', ConfidenceLevel, 'style', StyleGT);
%     
%     legend(h, Lable{nLable});    
%     
%     axis equal
%     xlim([-Range, Range].*1.5);
%     ylim([-Range, Range]);
% 
%     set(gca,'FontSize',Font.Size);
%     set(gca,'LineWidth',Axis.Width);
%     set(gca,'layer','top') ;
%     set(gcf, 'Color', 'w');    
%     hold off
% end
% hold off

end

