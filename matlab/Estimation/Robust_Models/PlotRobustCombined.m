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
hError = figure;
hold on
for n = 1:numel(Metric)
    hHist = histogram(abs(Metric(n).Error), 'BinWidth', 0.1);
    hHist.DisplayName = Metric(n).Lable;
    hHist.Normalization = 'pdf';
    hHist.BinEdges = hHist.BinEdges - hHist.BinWidth/2.0;
    hHist.LineStyle = 'none';
    hHist.FaceColor = Config.Line.Color(n,:);
end
hold off
% Format
set(gca, 'YScale', 'log')
% axis tight
% xlim([-0.1 inf])
xlabel('Estimation Error');
ylabel('Probability');
legend('show', 'Location','best');
plot.formatPlot(hError, Config);
if(SavePlot)
    plot.exportPlot(hError, PlotPath, [PlotPrefix 'ErrorHistComb']);
end

%% plot estimation error boxplot
hErrorBox = plot.createBoxPlot([Metric(:).Error], {Metric(:).Lable}, Config);
ylabel('Estimation Error');
plot.formatPlot(hErrorBox, Config);
if(SavePlot)
    plot.exportPlot(hErrorBox, PlotPath, [PlotPrefix 'ErrorBoxComb']);
end

%% plot runtime
hRuntime = figure;
hold on
for n = 1:numel(Metric)
    hHist = histogram(Metric(n).Duration*1e3, 'BinWidth', 0.01);
    hHist.DisplayName = Metric(n).Lable;
    hHist.Normalization = 'pdf';
    hHist.BinEdges = hHist.BinEdges - hHist.BinWidth/2.0;
    hHist.LineStyle = 'none';
    hHist.FaceColor = Config.Line.Color(n,:);
end
hold off
% Format
xlabel('Runtime [µs]');
ylabel('Probability');
legend('show', 'Location','best');
plot.formatPlot(hRuntime, Config);
if(SavePlot)
    plot.exportPlot(hRuntime, PlotPath, [PlotPrefix 'RuntimeComb']);
end

end

