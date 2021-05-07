function [] = PlotRobust2D(ErrorModels, GMM, Result, Metric, SavePlot)
%PLOTROBUST2D Summary of this function goes here
%   Detailed explanation goes here
%% config plot
Config = plot.loadPlotConfig;

if(SavePlot)
    PlotPath = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    PlotPath = [PlotPath '/Results/Plots/GMM/'];
    if isfolder(PlotPath) == false
        mkdir(PlotPath)
    end
end

Axis.XLable = 'Residual in x';
Axis.YLable = 'Residual in y';
Axis.ZLable = 'Cost';

% custom labels
Lables = ["Max-Mixture", "Sum-Mixture",  "Max-Sum-Mixture"];
Width = 900;

%% preprocessing
% get result properties
NumberPoints = sqrt(size(Result.Error,1));
NumberAlgo = numel(Result.Raw);
RangeSurface = max(GMM.Samples(:,1)) - min(GMM.Samples(:,1));

% convert to redable formate
ErrorX = Result.Error(:,1,1);
ErrorY = Result.Error(:,2,1);
ProbGMM = GMM.Prob;
CostAlgo = Result.Cost;

% reshape to NxN array
ArrayErrorX = reshape(ErrorX, [NumberPoints NumberPoints])';
ArrayErrorY = reshape(ErrorY, [NumberPoints NumberPoints])';
ArrayCostGMM = reshape(-log(ProbGMM), [NumberPoints NumberPoints]);
for n = 1:NumberAlgo
    ArrayCostAlgo(:,:,n) = reshape(CostAlgo(:,n), [NumberPoints NumberPoints]);
end

% create line
LineResolution = 0.1;
LineX = min(ErrorX):LineResolution:max(ErrorX);
LineY = (min(ErrorY):LineResolution:max(ErrorY))/2;

% interpolate Line
for n = 1:NumberAlgo
    LineCostAlgo(:,n) = interp2(ArrayErrorX(),ArrayErrorY,...
                                squeeze(ArrayCostAlgo(:,:,n)),...
                                LineX,LineY,'makima');
end

%% plot surface
hError = figure;
tiledlayout(3,2,'TileSpacing','none', 'Padding', 'none');
hTile1 = nexttile([2 2]);

% find levels
Min = min(ArrayCostGMM,[],'all');
Max = max(ArrayCostGMM,[],'all');
Range = Max - Min;
Levels = (0:0.05:1).^1.2 * Range + Min;

hold on
[~, hContour] = contourf(ArrayErrorX, ArrayErrorY, ArrayCostGMM, Levels,'-', 'LineWidth', 0.1, 'LineStyle','none');
contour(ArrayErrorX, ArrayErrorY, ArrayCostGMM, Levels,'-', 'LineWidth',1, 'HandleVisibility','off');
hContour.LineColor = 'none';
hContour.DisplayName = 'Neg. Log-Likelihood';
brighten(0.6);
shading flat

%% plot optimized points
for n = 1:NumberAlgo
    % remove boarder points for nicer plot
    InBound = abs(Result.PostOpt(:,1,n)) < (RangeSurface/2 - 0.08) &...
              abs(Result.PostOpt(:,2,n)) < (RangeSurface/4 - 0.08) &...
              rand(size(Result.PostOpt(:,1,n))) < 100/(NumberPoints*NumberPoints);
    
    hScatter{n} = plot(-Result.PostOpt(InBound,1,n), -Result.PostOpt(InBound,2,n),'o', 'LineWidth', 1.25);
    hScatter{n}.MarkerSize = Config.Line.Marker.Size;
    hScatter{n}.MarkerEdgeColor = 'k';
    hScatter{n}.MarkerFaceColor = Config.Color.Default(n,:);
    
    set(hScatter{n}, 'DisplayName', Lables(n));
end

% plot global optimum
%hOpt = plot(-GMM.GlobalMax(1), -GMM.GlobalMax(2),'+', 'MarkerSize', Config.Line.Marker.Size, 'LineWidth', Config.Line.Width, 'Color', [0.8 0.8 0.0], 'HandleVisibility','off');
%set(hOpt, 'DisplayName', 'Global Optimum');

% plot slicing line
hSliceLine = plot(LineX, LineY,'--', 'LineWidth', Config.Line.Width, 'Color', [0.2 0.2 0.2]);
hSliceLine.DisplayName = 'Line of Intersection';
hold off

% formate first tile
%hLedgend = legend(hTile1, [hContour hScatter{:}]);
legend(hTile1, 'Location','southeast');
axis equal
box on

%xlabel(Axis.XLable);
ylabel(Axis.YLable);

xticks([min(ArrayErrorX,[],'all'):2:max(ArrayErrorX,[],'all')]);
yticks([min(ArrayErrorY,[],'all'):2:max(ArrayErrorY,[],'all')]);
xticklabels({});

xlim([min(ArrayErrorX,[],'all') max(ArrayErrorX,[],'all')]);
ylim([min(ArrayErrorY,[],'all') max(ArrayErrorY,[],'all')]./2);

uistack(hScatter{1},'top');
uistack(hScatter{3},'top');
uistack(hSliceLine,'top');

plot.formatPlot(hError, plot.loadPlotConfig([Width Width/1.5*2]));

%% plot sliced surface
nexttile([1 2])
hold on
for n = 1:NumberAlgo
    hCostSlice{n} = plot(LineX, LineCostAlgo(:,n),'-', 'LineWidth', Config.Line.Width, 'Color', Config.Color.Default(n,:));
    hCostSlice{n}.DisplayName = Lables(n);
end
hold off

% formate second plot
grid on
xticks([min(ArrayErrorX,[],'all'):2:max(ArrayErrorX,[],'all')]);
yticks([floor(min(LineCostAlgo,[],'all')):4:ceil(max(LineCostAlgo,[],'all'))]);


xlim([min(ArrayErrorX,[],'all') max(ArrayErrorX,[],'all')]);
ylim([min(LineCostAlgo,[],'all')-0.5 ceil(max(LineCostAlgo,[],'all'))-3]);

xlabel(Axis.XLable);
ylabel('Intersected Cost');
box on

uistack(hCostSlice{2},'bottom');

% Format automatic
plot.formatPlot(hError, plot.loadPlotConfig([Width Width/1.25]));
if(SavePlot)
    plot.exportPlot(hError, PlotPath, 'Error2D');
end

%% plot slice through surface 

%% plot estimation error histogram
hErrorHist = figure;
hold on
for n = 1:numel(ErrorModels)
    hHist = histogram(Metric(n).Error, 'BinWidth', 0.025);
    hHist.DisplayName = ErrorModels{n};
    hHist.Normalization = 'pdf';
    hHist.BinEdges = hHist.BinEdges - hHist.BinWidth/2.0;
    hHist.LineStyle = 'none';
    hHist.FaceColor = Config.Color.Default(n,:);
end
hold off
% Format
xlabel('Estimation Error');
ylabel('Probability');
legend('show', 'Location','best');
plot.formatPlot(hErrorHist, Config);
if(SavePlot)
    plot.exportPlot(hErrorHist, PlotPath, 'ErrorHist1D');
end

%% plot runtime
hRuntime = figure;
hold on
for n = 1:numel(ErrorModels)
    hHist = histogram(Metric(n).Duration*1e3, 'BinWidth', 0.01);
    hHist.DisplayName = ErrorModels{n};
    hHist.Normalization = 'pdf';
    hHist.BinEdges = hHist.BinEdges - hHist.BinWidth/2.0;
    hHist.LineStyle = 'none';
    hHist.FaceColor = Config.Color.Default(n,:);
end
hold off
% Format
xlabel('Runtime [µs]');
ylabel('Probability');
legend('show', 'Location','best');
plot.formatPlot(hRuntime, Config);
if(SavePlot)
    plot.exportPlot(hRuntime, PlotPath, 'Runtime2D');
end

%% cost surface plot
% for n = 1:numel(ErrorModels)
%     % convert To matrices
%     CostZ = reshape(Result.Cost(:,n), [Points Points]);
%     GradientX = reshape(Result.Gradient(:,1,n), [Points Points]);
%     GradientY = reshape(Result.Gradient(:,2,n), [Points Points]);
%     
%     % plot cost
%     hCost = figure;
%     hSurf = surf(ErrorX,ErrorY, CostZ,'FaceColor','interp','EdgeColor','none','FaceLighting','gouraud');
%     set(hSurf, 'DisplayName', ['Cost - ' ErrorModels{n}]);
%     legend('show');
%     
%     xlabel(Axis.XLable);
%     ylabel(Axis.YLable);
%     zlabel(Axis.ZLable);
%     plot.formatPlot(hCost, Config);
%     
%     % plot gradient
%     hGradient = figure;
%     contour(ErrorX, ErrorY, CostZ);
%     hold on
%     quiver(ErrorX, ErrorY, -GradientX, -GradientY);
%     quiver(ErrorX, ErrorY, DY, DX);
%     plot(Result.PostOpt(1,1,n), Result.PostOpt(1,2,n),'x', 'MarkerSize', Config.Line.Marker.Size, 'LineWidth', Config.Line.Width*2, 'Color', Config.Color.Default(1,:), 'HandleVisibility','off');
%     hold off
%     
%     legend({['Cost - ' ErrorModels{n}], ['Gradient - ' ErrorModels{n}], ['Gradient Numerical  - ' ErrorModels{n}]});
%     xlabel(Axis.XLable);
%     ylabel(Axis.YLable);
%     plot.formatPlot(hGradient, Config);
%     
% end

%% Hessian surf plot
hHessian = figure;
tiledlayout(1,numel(ErrorModels));
for n = 1:numel(ErrorModels)
    
    nexttile
    
    Hessian = reshape(Result.HessianNorm(:,n), [NumberPoints NumberPoints]);
    X = reshape(Result.Raw(n).Cost2.Position(:,1), [NumberPoints NumberPoints]);
    Y = reshape(Result.Raw(n).Cost2.Position(:,2), [NumberPoints NumberPoints]);
    
    % plot cost
%     imagesc(Result.Raw(1).Cost2.Position(:,1), Result.Raw(1).Cost2.Position(:,2),  Hessian, [0 max(Result.HessianNorm, [], 'all')]);
    surf(X, Y, Hessian, 'FaceColor','interp', 'EdgeColor','none', 'FaceLighting','flat');
    
    xlabel(Axis.XLable);
    ylabel(Axis.YLable);
    zlabel('Hessian Norm');
    
    xlim([min(Result.Raw(1).Cost2.Position(:,1)) max(Result.Raw(1).Cost2.Position(:,1))])
    ylim([min(Result.Raw(1).Cost2.Position(:,2)) max(Result.Raw(1).Cost2.Position(:,2))])
    
    caxis([0 max(Result.HessianNorm, [], 'all')])
    
    title(Lables(n));
    
    axis equal
    axis tight
    
    colormap jet
end
c = colorbar;
c.Label.String = 'Norm of Hessian';
plot.formatPlot(hHessian, plot.loadPlotConfig([1800 600]));
if(SavePlot)
    plot.exportPlot(hHessian, PlotPath, 'Hessian2D_3D');
end

%% Hessian img plot
hHessian = figure;
tiledlayout(1,numel(ErrorModels));
for n = 1:numel(ErrorModels)
    
    nexttile
    
    Hessian = reshape(Result.HessianNorm(:,n), [NumberPoints NumberPoints]);
    
    % plot cost
    imagesc(Result.Raw(1).Cost2.Position(:,1), Result.Raw(1).Cost2.Position(:,2),  Hessian);
    
    xlabel(Axis.XLable);
    ylabel(Axis.YLable);
    zlabel('Hessian Norm');
    
    xlim([min(Result.Raw(1).Cost2.Position(:,1)) max(Result.Raw(1).Cost2.Position(:,1))])
    ylim([min(Result.Raw(1).Cost2.Position(:,2)) max(Result.Raw(1).Cost2.Position(:,2))])
    
    caxis([0 max(Result.HessianNorm, [], 'all')])
    
    title(Lables(n));
    
    axis equal
    axis tight
    
    colormap jet
end
c = colorbar;
c.Label.String = 'Norm of Hessian';
plot.formatPlot(hHessian, plot.loadPlotConfig([1800 600]));
if(SavePlot)
    plot.exportPlot(hHessian, PlotPath, 'Hessian2D_Flat');
end

end

