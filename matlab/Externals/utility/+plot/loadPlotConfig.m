function [Config] = loadPlotConfig(Size)
%LOADPLOTOPTIONS Summary of this function goes here
%   Detailed explanation goes here

Config.Font.Size = 18;
Config.Font.Latex = false;
Config.Font.Name = 'Helvetica';

Config.Line.Width = 3;

% overwrite options
Config.Overwrite.Colormap = true;
Config.Overwrite.Lines = false;

% set default colors
if exist('brewermap','file') > 0
    % nice colors from: https://github.com/DrosteEffect/BrewerMap
    Config.Line.Color = brewermap(5, 'Set1');
    Config.Line.Color = brighten(Config.Line.Color([1 3 2 5 4],:), -0.2);
    
    Config.Colormap = flipud(brewermap(1024, 'YlGnBu'));
    Config.ColormapDiff = brewermap(1024, 'RdBu');
else
    % older default colors
    warning('ColorBrewer not available, use old default colors!');
    Config.Line.Color = [[0.8 0 0]; [0 0.7 0]; [0 0 0.6]; [0.9 0.4 0]; [0.4 0 0.9]];
    Config.Colormap = parula(1024);
    Config.ColormapDiff = turbo(1024);
end

Config.Line.Color = [Config.Line.Color; brighten(Config.Line.Color, 0.7)];
Config.Line.ColorGrey = [0.35 0.35 0.35];
Config.Line.Marker.Type = {'x', '+', 'o', '*','v', 's', 'd','p'};
Config.Line.Marker.Size = 15;

Config.Axis.Grid.Style = ':';
Config.Axis.XLable = 'X';
Config.Axis.YLable = 'Y';
Config.Axis.Width = 2;

if nargin < 1
    Config.Plot.Size = [1600 1200];
else
    Config.Plot.Size = Size;
end

end

