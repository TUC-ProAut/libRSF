function [] = formatPlot(hFig, Config)
%FORMATPLOT Summary of this function goes here
%   Detailed explanation goes here

    %% find relevant handles
    hAxis = findall(hFig,'type','axes');
    hColorbar = findobj(hFig, 'Type', 'Colorbar');
    hLine = findall(hFig,'type','line');
    hFontSize = findall(hFig,'-property','FontSize');
    hFontName = findall(hFig,'-property','FontName');

    %% format lines
    if Config.Overwrite.Lines == true
        set(hLine,'LineWidth',Config.Line.Width);
    end
    
    %% format text
    set(hFontSize,'FontSize',Config.Font.Size);
    set(hFontName,'FontName',Config.Font.Name);
    if Config.Font.Latex == true
        set(hFig,'defaulttextinterpreter','latex')
    else
        set(hFig,'defaulttextinterpreter','none')
    end

    %% format axes
    set(hAxis,'LineWidth',Config.Axis.Width);
    set(hAxis,'GridLineStyle',Config.Axis.Grid.Style);
    set(hAxis,'layer','top');

    if isfield(Config.Axis,'YLimit')
        ylim(hAxis, Config.Axis.YLimit);
    end    

    %% reset legend for right box size
    legend toggle
    legend toggle

    %% format colorbar
    set(hColorbar,'LineWidth',Config.Axis.Width);

    %% set better colormap
    if Config.Overwrite.Colormap == true
        colormap(hFig, Config.Colormap);
    end

    %% format overall plot
    if isprop(hFig, 'Color')
        set(hFig, 'Color', 'w');
    end
    set(hFig, 'Position', [0 0 Config.Plot.Size]);
end

