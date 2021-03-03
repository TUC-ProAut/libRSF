function [] = evaluateErrorHistogram(Lables, ErrorMatrix)
%EVALUATEERRORHISTOGRAM Summary of this function goes here
%   Detailed explanation goes here

%% config plot
Font.Size = 20;
Line.Width = 2;
Line.Color = [[0.5 0.0 0.0]; [0.8 0.0 0.0]; [0.0 0.5 0.0];...
              [0.0 0.8 0.0]; [0.0 0.0 0.5]; [0.0 0.0 0.8];...
              [0.0 0.5 0.5]; [0.5 0.0 0.5]; [0.5 0.5 0.0];...
              [0.0 0.8 0.8]; [0.8 0.0 0.8]; [0.8 0.8 0.0]];
Line.Marker.Type = {'none'; 'v'; '+'; 'x'; 's'};
Line.Marker.Size = 14;

Axis.Width = 2;

Legend = {'LOS', 'NLOS'};

Ylable = {'Mean [m]', 'StdDev [m]', 'Weight'};

Plot.Size = [1600 800];

BinWidth = 1.0;

%% plot
figure;
hold on
for nLable = 1:length(Lables)
    h = histogram(ErrorMatrix(nLable,:));
    
    h.DisplayName = Lables{nLable};
    h.Normalization = 'pdf';
    h.BinWidth = BinWidth;
    h.BinEdges = h.BinEdges + h.BinWidth/2;
    h.LineStyle = 'none';
    
    set(gca,'FontSize',Font.Size);
    set(gca,'LineWidth',Axis.Width);
    set(gca,'layer','top');
    set(gcf, 'Color', 'w');
end
legend show
hold off
    

end

