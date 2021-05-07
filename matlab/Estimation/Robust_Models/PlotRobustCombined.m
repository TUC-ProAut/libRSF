function [] = PlotRobustCombined(Metric, PlotPrefix, SavePlot)
%PLOTROBUSTCOMBINED1D Summary of this function goes here
%   Detailed explanation goes here

%% config
Config = plot.loadPlotConfig;

if(SavePlot)
    PlotPath = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    PlotPath = [PlotPath '/Results/Plots/GMM/'];
    if isfolder(PlotPath) == false
        mkdir(PlotPath)
    end
end

%% plot estimation error histogram
BinWidth = max([Metric.Error], [], 'all')/100;
hError = figure;
hold on
for n = 1:numel(Metric)
    hHist = histogram(abs(Metric(n).Error), 'BinWidth', BinWidth);
    hHist.DisplayName = Metric(n).Lable;
    hHist.Normalization = 'pdf';
    hHist.BinEdges = hHist.BinEdges - hHist.BinWidth/2.0;
    hHist.LineStyle = 'none';
    hHist.FaceColor = Config.Color.Default(n,:);
end
hold off

% custom format
box on
set(gca, 'YScale', 'log')
legend('show', 'Location','best');
title('Distribution of Estimation Error');
xlabel('Estimation Error [m]');
ylabel('Probability');

%% plot estimation error boxplot
hErrorBox = plot.createBoxPlot([Metric(:).Error], {Metric(:).Lable}, Config);

% custom format
ylabel('Estimation Error [m]');
title('Estimation Error Boxplot');

%% plot runtime histogram
BinWidth = max([Metric.Duration], [], 'all')/100;
hRuntime = figure;
hold on
for n = 1:numel(Metric)
    hHist = histogram(Metric(n).Duration*1e3, 'BinWidth', BinWidth*1e3);
    hHist.DisplayName = Metric(n).Lable;
    hHist.Normalization = 'pdf';
    hHist.BinEdges = hHist.BinEdges - hHist.BinWidth/2.0;
    hHist.LineStyle = 'none';
    hHist.FaceColor = Config.Color.Default(n,:);
end
hold off

% custom format
box on
set(gca, 'YScale', 'log')
legend('show', 'Location','best');
xlabel('Runtime [ms]');
ylabel('Probability');
title('Distribution of Computation Time');

%% format all plots
plot.formatPlot(hError, Config);
plot.formatPlot(hErrorBox, Config);
plot.formatPlot(hRuntime, Config);

%% save all plots
if(SavePlot)
    plot.exportPlot(hError, PlotPath, [PlotPrefix 'ErrorHistComb']);
    plot.exportPlot(hErrorBox, PlotPath, [PlotPrefix 'ErrorBoxComb']);
    plot.exportPlot(hRuntime, PlotPath, [PlotPrefix 'RuntimeComb']);
end

end

