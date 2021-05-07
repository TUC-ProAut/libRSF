function [hFig] = createPlotGeneric(XData, YData, Lables, XLable, YLable)
%GENERICPLOT Summary of this function goes here
%   Detailed explanation goes here

%% config plot
Config = plot.loadPlotConfig();

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

%xlim([-0.1 2.5]);
%ylim([-0.1 2.5]);
%axis tight
% axis equal
grid on
box on

legend('show')
legend('Location','best')

plot.formatPlot(hFig, Config);

end

