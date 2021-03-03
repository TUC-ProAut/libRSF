function [] = evaluateGMM(GMM, GMM_GT)
%EVALUATE2D Summary of this function goes here
%   Detailed explanation goes here

PlotMat = GMM(:,2:end);
TimeVect = GMM(:,1);

PlotMat_GT = GMM_GT(:,2:end);
TimeVect_GT = GMM_GT(:,1);

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

%% 2D plot
figure
hold on
for nEle = 1:6
    sub = subplot(3, 2, nEle);
    
    hold on
    plot(sub,TimeVect_GT,PlotMat_GT(:,nEle),'LineWidth',Line.Width,'Color',Line.Color(nEle,:).*0.0);
    plot(sub,TimeVect,PlotMat(:,nEle),'LineWidth',Line.Width,'Color',Line.Color(nEle,:));
    hold off
    
    %set axis
    if mod(nEle,2) == 1
        ylabel(Ylable{ceil(nEle/2)})
    end
    
    %set LOS/NLOS
    if nEle < 3
        title(sub,Legend{nEle})
    end 
    
    xlim([min(TimeVect) max(TimeVect)]);
    
    set(gca,'FontSize',Font.Size);
    set(gca,'LineWidth',Axis.Width);
    set(gca,'layer','top');
    set(gcf, 'Color', 'w');
end

set(gcf, 'Position', [0 0 Plot.Size]);
hold off

%export_fig('-painters','GMM.eps')

end

