function [] = formatPlot(hFig, Config)
%FORMATPLOT Summary of this function goes here
%   Detailed explanation goes here

    %% find relevant handles
    hAxis = findall(hFig,'type','axes');
    hLeg = findall(hFig,'type','legend');
    hColorbar = findobj(hFig, 'Type', 'Colorbar');
    hLine = findall(hFig,'type','line');
    hFontSize = findall(hFig,'-property','FontSize');
    hFontName = findall(hFig,'-property','FontName');

    %% format lines
    if Config.Overwrite.Lines == true
        set(hLine,'LineWidth',Config.Line.Width);
    end
    
    %% format text
    if Config.Overwrite.Fontsize
        set(hFontSize,'FontSize',Config.Font.Size);
    end
    
    if Config.Font.Latex == true
        set(hFig,'defaulttextinterpreter','latex')
    else
        set(hFig,'defaulttextinterpreter','none')
        set(hFontName,'FontName',Config.Font.Name);
    end

    %% format axes
    set(hAxis,'LineWidth',Config.Axis.Width);
    set(hAxis,'GridLineStyle',Config.Axis.Grid.Style);
    set(hAxis,'layer','top');
    set(hAxis, 'XColor', Config.Axis.Color, 'YColor', Config.Axis.Color)

    if isfield(Config.Axis,'YLimit')
        ylim(hAxis, Config.Axis.YLimit);
    end
    
    %% format legend
    set(hLeg, 'EdgeColor', Config.Legend.Color);

    % reset legend for right box size
    legend toggle
    legend toggle

    %% format colorbars
    for n = 1:numel(hColorbar)
        set(hColorbar(n),'LineWidth',Config.Axis.Width);
        set(hColorbar(n),'Color',Config.Axis.Color);
        if isprop(hColorbar(n), 'Label')
            hColorbar(n).Label.Color = Config.Axis.Color;
        end
    end

    %% set better colormap
    if Config.Overwrite.Colormap == true
        colormap(hFig, Config.Colormap.Default);
    end

    %% format overall plot
    % white background
    if isprop(hFig, 'Color')
        set(hFig, 'Color', 'w');
    end
    % force Matlab to apply all the changes
    drawnow
    % set size in pixels
    set(hFig , 'units', 'pixels', 'position', [0 0 Config.Plot.Size]);
end

