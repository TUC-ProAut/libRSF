function [] = PlotRobustCombined(SaveFile, PlotPath, SavePlot)
%PLOTROBUSTCOMBINED1D Summary of this function goes here
%   Detailed explanation goes here

load(SaveFile, "-mat", "CombinedMetric", "BaseName", "Config");

%% config
PlotConfig = loadThesisConfig(0.5, 0.3);

%% analyze
MaxError = 20;% max(max([CombinedMetric.Error], [], 'all'), 1e-2);
MaxRuntime = 40;%max([CombinedMetric.Duration], [], 'all')*1e3;
NumPoints = length(CombinedMetric(1).Duration);
MinProb = 1 / NumPoints;

%% plot estimation error histogram
BinWidth = MaxError/100;
hError = figure;
hold on
for n = 1:numel(CombinedMetric)
    hHist = histogram(abs(CombinedMetric(n).Error), 'BinWidth', BinWidth);
    hHist.DisplayName = Config.Labels{n};
    hHist.Normalization = 'probability';
    hHist.BinEdges = hHist.BinEdges - hHist.BinWidth/2.0;
    hHist.LineStyle = 'none';
    hHist.FaceColor = PlotConfig.Color.Default(n,:);
end
hold off

% custom format
box on
set(gca, 'YScale', 'log')
legend('show', 'Location','northeast');
title('Accuracy');
xlabel('Error [m]');
ylabel('Probability');
xlim([-MaxError/200 MaxError]);
ylim([1e-6 1.1]);

%% plot estimation error boxplot
hErrorBox = plot.createBoxPlot(abs([CombinedMetric(:).Error]), Config.Labels, PlotConfig);

% create title
Title = num2str(Config.NumDim) + "D ";
if Config.SymmetricGMM
    Title = Title + "Symmetric";
else
    Title = Title + "Asymmetric";
end

% custom format
ylim([-1 25]);
ylabel('Error [m]');
title(Title);

%% plot runtime histogram
BinWidth = MaxRuntime/100;
hRuntime = figure;
hold on
for n = 1:numel(CombinedMetric)
    hHist = histogram(CombinedMetric(n).Duration*1e3, 'BinWidth', BinWidth);
    hHist.DisplayName = Config.Labels{n};
    hHist.Normalization = 'probability';
    hHist.BinEdges = hHist.BinEdges - hHist.BinWidth/2.0;
    hHist.LineStyle = 'none';
    hHist.FaceColor = PlotConfig.Color.Default(n,:);
end
hold off

% custom format
box on
set(gca, 'YScale', 'log')
legend('show', 'Location','northeast');
xlabel('Runtime [ms]');
ylabel('Probability');
title('Efficiency');
xlim([-MaxRuntime/200 MaxRuntime]);
ylim([MinProb 1.1]);

%% format all plots
plot.formatPlot(hError, PlotConfig);
plot.formatPlot(hErrorBox, PlotConfig);
plot.formatPlot(hRuntime, PlotConfig);

%% save all plots
if(SavePlot)
    % create path
    if isfolder(PlotPath) == false
        mkdir(PlotPath)
    end

    % save plots
    plot.exportPlot(hError, PlotPath, [BaseName 'ErrorHist']);
    plot.exportPlot(hErrorBox, PlotPath, [BaseName 'ErrorBox']);
    plot.exportPlot(hRuntime, PlotPath, [BaseName 'RuntimeHist']);
end

end

