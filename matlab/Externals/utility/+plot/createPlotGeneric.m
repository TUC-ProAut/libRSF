function [hFig] = createPlotGeneric(XData, YData, Lables, XLable, YLable)
%GENERICPLOT Summary of this function goes here
%   Detailed explanation goes here

%% config plot
Config = plot.loadPlotConfig();

% bigger text
Config.Font.Size = 20;

% thicker lines
Config.Overwrite.Lines = true;
Config.Line.Width = 3;
Config.Axis.Width = 2;

% grey color
Config.Axis.Color = Config.Color.Grey;
Config.Legend.Color = Config.Color.Grey;

%% 2D plot
hFig = figure;

hold on
for nLable =1:length(Lables)
    p = plot(XData(nLable,:), YData(nLable,:),'LineWidth',Config.Line.Width,'Color',Config.Color.Default(nLable,:));
    p.DisplayName = Lables{nLable};
end
hold off

xlabel(XLable);
ylabel(YLable);

xlim([min(XData,[],'all'), max(XData,[],'all')])
grid on
box on

legend('show')
legend('Location','best')

plot.formatPlot(hFig, Config);

end

