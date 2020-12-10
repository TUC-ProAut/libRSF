function [] = evaluateClock(Clock, ClockGT, Lables)
%EVALUATE2D Summary of this function goes here
%   Detailed explanation goes here


%% config plot
Font.Size = 20;
Line.Width = 2;
Line.Color = [[0.5 0.0 0.0]; [0.0 0.5 0.0]; [0.0 0.0 0.5];...
              [0.8 0.0 0.0]; [0.0 0.8 0.0]; [0.0 0.0 0.8];...
              [0.0 0.5 0.5]; [0.5 0.0 0.5]; [0.5 0.5 0.0];...
              [0.0 0.8 0.8]; [0.8 0.0 0.8]; [0.8 0.8 0.0]];
Line.Marker.Type = {'none'; 'v'; '+'; 'x'; 's'};
Line.Marker.Size = 14;

Axis.XLable = 'Time [s]';
Axis.YLable = 'Clock Error [m]';
Axis.Width = 2;

Plot.Size = [1600 800];

%% 2D plot
figure
hold on
for nLable = 1:length(Lables)
    p = plot(Clock(nLable,:,1),Clock(nLable,:,2)-ClockGT,'LineWidth',Line.Width,'Color',Line.Color(nLable,:));
    p.DisplayName = Lables{nLable};    
end

legend show
set(gca,'FontSize',Font.Size);
set(gca,'LineWidth',Axis.Width);
set(gca,'layer','top');
set(gcf, 'Color', 'w');

xlabel(Axis.XLable);
ylabel(Axis.YLable);

set(gcf, 'Position', [0 0 Plot.Size]);
hold off

end

