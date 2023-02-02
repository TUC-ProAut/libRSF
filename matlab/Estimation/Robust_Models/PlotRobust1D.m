function [] = PlotRobust1D(ErrorModels, GMM, Result, Metric, SavePlot)
%PLOTROBUST1D Summary of this function goes here
%   Detailed explanation goes here
% config plot
Config = plot.loadPlotConfig;

if(SavePlot)
    PlotPath = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    PlotPath = [PlotPath '/Results/Plots/GMM/'];
    if isfolder(PlotPath) == false
        mkdir(PlotPath)
    end
end

Axis.XLabel = 'Error';
Axis.YLabel1 = 'Cost';
Axis.YLabel2 = 'Gradient';
Axis.YLabel3 = 'Hessian';

%% plot cost
% cost
hCost = figure;
subplot(3,1,1)
for n = 1:numel(ErrorModels)
    hLine = plot(Result.Error(:,n), Result.Cost(:,n) - min(Result.Cost(:,n)), 'LineWidth', Config.Line.Width, 'Color', Config.Color.Default(n,:));
    set(hLine, 'DisplayName', ErrorModels{n});
    hold on
end
hold off

legend('show');
xlabel(Axis.XLabel);
ylabel(Axis.YLabel1);
ylim([0 5]);

% gradient
subplot(3,1,2)
for n = 1:numel(ErrorModels)
    h = plot(Result.Error(:,n), Result.Gradient(:,n), 'LineWidth', Config.Line.Width, 'Color', Config.Color.Default(n,:));
    set(h, 'DisplayName', ErrorModels{n});
    hold on
end
hold off

legend('show');
xlabel(Axis.XLabel);
ylabel(Axis.YLabel2);


% Hessian
subplot(3,1,3)
for n = 1:numel(ErrorModels)
    h = plot(Result.Error(:,n), Result.Hessian(:,n), 'LineWidth', Config.Line.Width, 'Color', Config.Color.Default(n,:));
    set(h, 'DisplayName', ErrorModels{n});
    hold on
end
h = plot(Result.Error(:,1), GMM.Hessian, '--', 'LineWidth', Config.Line.Width, 'Color', [0 0 0]);
set(h, 'DisplayName', 'Numerical Hessian');
hold off

legend('show');
xlabel(Axis.XLabel);
ylabel(Axis.YLabel3);
axis tight
%xlim([min(Result.Error(:,1)) max(Result.Error(:,1))]);
plot.formatPlot(hCost, Config);
if(SavePlot)
    plot.exportPlot(hCost, PlotPath, 'Cost1D');
end


%% plot probability
hProb = figure;
for n = 1:numel(ErrorModels)
    hLine = plot(Result.Error(:,n), Result.Prob(:,n), 'LineWidth', Config.Line.Width, 'Color', Config.Color.Default(n,:));
    set(hLine,'DisplayName',ErrorModels{n});
    hold on
end
hold off
legend('show');

plot.formatPlot(hProb, Config);
if(SavePlot)
    plot.exportPlot(hProb, PlotPath, 'Prob1D');
end

%% plot optimizazion results
hOpt = figure;
hold on
for n = 1:numel(ErrorModels)
    hLine = plot(Result.Error(:,n), Result.Cost(:,n), 'LineWidth', Config.Line.Width, 'Color', Config.Color.Default(n,:));
    set(hLine,'DisplayName',ErrorModels{n});
    plot(-Result.PostOpt(:,n), ones(size(Result.PostOpt(:,n)))*n + min(Result.Cost,[],'all'), 'x', 'MarkerSize', Config.Line.Marker.Size, 'LineWidth', Config.Line.Width, 'Color', Config.Color.Default(n,:), 'HandleVisibility','off')
end
hold off
legend('show');

plot.formatPlot(hOpt, Config);
if(SavePlot)
    plot.exportPlot(hOpt, PlotPath, 'Opt1D');
end

%% plot estimation error histogram
hError = figure;
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
plot.formatPlot(hError, Config);
if(SavePlot)
    plot.exportPlot(hError, PlotPath, 'ErrorHist1D');
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
    plot.exportPlot(hRuntime, PlotPath, 'Runtime1D');
end
end

