function [Config] = loadPlotConfig(Size)
%LOADPLOTOPTIONS Summary of this function goes here
%   Detailed explanation goes here

% figure size in pixel
if nargin < 1
    Config.Plot.Size = [1600 1200];
else
    Config.Plot.Size = Size;
end

%font options
Config.Font.Size = 18;
Config.Font.Latex = false;

% select font
FontPriority = {'Verdana', 'TeXGyreHeros', 'Helvetica', 'Liberation Sans'};
for n = 1:numel(FontPriority)
    if any(strcmp(listfonts, FontPriority{n})) 
        Config.Font.Name = FontPriority{n};
        break
    elseif any(strcmp(listTrueTypeFonts, FontPriority{n})) 
        Config.Font.Name = FontPriority{n};
        break
    else
        warning(['The font ' FontPriority{n} ' ist not available!']);
    end
end
if isfield(Config.Font, 'Name') == false
    Config.Font.Name = get(0,'defaultAxesFontName');
end

% overwrite options
Config.Overwrite.Colormap = true;
Config.Overwrite.Lines = false;
Config.Overwrite.Fontsize = true;

% set default colors
if exist('brewermap','file') > 0
    % nice colors from: https://github.com/DrosteEffect/BrewerMap
    Config.Color.Default = brewermap(5, 'Set1');
    Config.Color.Default = brighten(Config.Color.Default([1 3 2 5 4],:), -0.2);
    
    Config.Colormap.Default = flipud(brewermap(1024, 'YlGnBu'));
    Config.Colormap.Differential = brewermap(1024, 'RdBu');
else
    % older default colors
    warning('ColorBrewer not available, use old default colors!');
    Config.Color.Default = [[0.8 0 0]; [0 0.7 0]; [0 0 0.6]; [0.9 0.4 0]; [0.4 0 0.9]];
    Config.Colormap.Default = parula(1024);
    Config.Colormap.Differential = turbo(1024);
end
Config.Color.Default = [Config.Color.Default; brighten(Config.Color.Default, 0.7); brighten(Config.Color.Default, -0.7)];

% add some custom colors
Config.Color.DarkGrey = [0.15 0.15 0.15];
Config.Color.Grey = [0.35 0.35 0.35];
Config.Color.LightGrey = [0.75 0.75 0.75];
Config.Color.TUC = [0 94 79]/255;
Config.Color.ETIT = [227 2 43]/255;

% line options
Config.Line.Width = 3;
Config.Line.Marker.Type = {'x', '+', 'o', '*','v', 's', 'd','p'};
Config.Line.Marker.Size = 15;

% axis options
Config.Axis.Grid.Style = ':';
Config.Axis.XLable = 'X';
Config.Axis.YLable = 'Y';
Config.Axis.Width = 2;
Config.Axis.Color = Config.Color.DarkGrey;

% legend options
Config.Legend.Color = Config.Axis.Color;

end

