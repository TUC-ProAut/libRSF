function [hBox] = createBoxPlot(Data, Lables, Config)
%BOXPLOT Summary of this function goes here
%   Detailed explanation goes here

if (size(Data, 2) > size(Data, 1)) && (size(Data, 1) == size(Lables, 2))
    Data = Data';
end

%% boxplot
hBox = figure;
b = boxplot(Data,Lables);

%% format
if isfield(Config.Axis,'YLimit')
    ylim(Config.Axis.YLimit);
end
ylabel(Config.Axis.YLable);

for nBox = 1:length(Lables)
    set(b(:,nBox),'LineWidth',Config.Line.Width);
end

if numel(Lables) > 5
    set(gca,'FontSize',10,'XTickLabelRotation',45)
end

box on
set(b,{'linew'},{Config.Line.Width});


%% set colors
set(findobj(hBox,'tag','Median'), 'Color', Config.Line.Color(1,:));
set(findobj(hBox,'tag','Box'), 'Color', Config.Line.Color(3,:));

h = findobj(hBox,'tag','Outliers');
for iH = 1:length(h)
    h(iH).MarkerEdgeColor = Config.Line.Color(1,:);
end

end

