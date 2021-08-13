function [ Handles ] = createPlot2D(Metric, Bonus, Animation)
%EVALUATE2D Summary of this function goes here
%   Detailed explanation goes here

if nargin < 3
    Animation = false;
end

%% parse input structs
M = length(Metric);
N = size(Metric(1).Estimate, 1);

% copy from struct
GT = Metric(1).GT;
ErrorRel = [];
ErrorRot = [];
for m=M:-1:1
    Trajectory(m,:,:) = Metric(m).Estimate;
    ErrorTrans(m,:) = Metric(m).TranslationalError;
    Lables{m} = Metric(m).Lable;
    
    if isfield(Metric, 'RelativeError')
        ErrorRel(m,:) = Metric(m).RelativeError;
    end
    if isfield(Metric, 'RotationalError')
        ErrorRot(m,:) = Metric(m).RotationalError;
    end
end

% check bonus information
HasUWBModule = false;
HasGNSSBase = false;
if nargin >1
    if isfield(Bonus,'Time')
        Timestamps = repmat(Bonus.Time, [1 M]);
    else
        Timestamps = ones(length(M),length(M)) .* [1:length(N)];
    end
    
    if isfield(Bonus,'Modules')
        HasUWBModule = true;
    end
    
    if isfield(Bonus,'Base')
        HasGNSSBase = true;
    end
end

%% config plot
PlotConfig = plot.loadPlotConfig;
PlotConfig.Axis.XLable = 'X [m]';
PlotConfig.Axis.YLable = 'Y [m]';

%% 2D plot
Handles{1} = figure;
hold on

% plot UWB modules (if available)
if HasUWBModule
    hPlotUWB = plot(Bonus.Modules.Position(:,1),Bonus.Modules.Position(:,2),'x','Color',[0.05 0.05 0.5],'MarkerSize',15,'LineWidth',3);
    hPlotUWB.DisplayName = Bonus.Modules.Lable;
    
    % add ID as text
    if isfield(Bonus.Modules,'ID')
        for n = 1:numel(Bonus.Modules.ID)
            text(Bonus.Modules.Position(n,1), Bonus.Modules.Position(n,2)+3,...
                 num2str(Bonus.Modules.ID(n)),...
                 'HorizontalAlignment','center', 'Color', [0 0 0.5], 'FontSize', PlotConfig.Font.Size);
        end
    end
end

% plot GNSS base (if available)
if HasGNSSBase
    hPlotBase = plot(Base.Position(:,1),Base.Position(:,2),'x','Color',[0 0 0],'MarkerSize',15,'LineWidth',3);
    hPlotBase.DisplayName = 'GNSS Base';
end

% plot ground truth
hPlotGT = plot(GT(:,1),GT(:,2),'LineWidth',PlotConfig.Line.Width,'Color',[0.0 0.0 0.0]);
hPlotGT.DisplayName = 'Ground Truth';

% plot trajectories
for m = 1:M
    hPlotTraj = plot(Trajectory(m,:,1),Trajectory(m,:,2),':','LineWidth',PlotConfig.Line.Width,'MarkerSize',15, 'Color',PlotConfig.Color.Default(m,:));
    hPlotTraj.DisplayName = Metric(m).Lable;
end
hold off

% formatting
xlabel(PlotConfig.Axis.XLable);
ylabel(PlotConfig.Axis.YLable);
axis equal
grid on
box on

% set boundary around GT
MinX = min(GT(:,1));
MaxX = max(GT(:,1));
MinY = min(GT(:,2));
MaxY = max(GT(:,2));
RangeX = MaxX - MinX;
RangeY = MaxY - MinY;
xlim([MinX MaxX] + [-RangeX RangeX]*0.1 + [-10 10]);
ylim([MinY MaxY] + [-RangeY RangeY]*0.1 + [-10 10]);

% find equal ticks
XT = xticks;
YT = yticks;
ResXT = median(diff(XT));
ResYT = median(diff(YT));
ResMax = max(ResXT, ResYT);
xticks(unique(round(XT/ResMax))*ResMax)
yticks(unique(round(YT/ResMax))*ResMax)

% add legend
legend('show', 'Location', 'southoutside', 'NumColumns', 2);

%% Boxplot
BoxConfig = PlotConfig;

% absolute translational Error
% BoxConfig.Axis.YLimit = [0 5];
BoxConfig.Axis.YLable = 'Absolute Error [m]';
Handles{2} = plot.createBoxPlot(ErrorTrans, Lables, BoxConfig);

% relative translational Error
if numel(ErrorRel) > 1
    BoxConfig.Axis.YLable = 'Relative Error [m]';
    Handles{end+1} = plot.createBoxPlot(ErrorRel, Lables, BoxConfig);
end

% rotational Error
% if numel(ErrorRot) > 1
%     BoxConfig.Axis.YLable = 'Rot. Error [°]';
%     Handles{end+1} = plot.createBoxPlot(ErrorRot, Lables, BoxConfig);
% end

%% runtime plots
if isfield(Metric, 'DurationTotal')
    
    for n = 1:numel(Metric)
        Size(n) = numel(Metric(n).DurationTotal);
    end
    
    if min(Size) > 1
        
        TimestampsDuration = Timestamps(1:numel(Metric(1).DurationTotal),:);
        
        % find maximum time
        MaximumDuration = ceil(max([Metric.DurationTotal], [], 'all')*100)*10;
        
        % cobined plot with total duration
        Handles{end+1} = plot.createPlotGeneric(TimestampsDuration', [Metric.DurationTotal]'.*1000,...
                                                {Metric.Lable}, 'Zeit [s]', 'Optimierungsdauer [ms]');                                    
        ylim([0 MaximumDuration]);
        xlim([min(TimestampsDuration,[],'all'), max(TimestampsDuration,[],'all')]);
        legend('Location','northwest')
        
        % find maximum iterations
        MaximumIterations = ceil(max([Metric.IterationSolver], [], 'all')/10)*10;
        
        % plot with solver iterations
        Handles{end+1} = plot.createPlotGeneric(TimestampsDuration', [Metric.IterationSolver]',...
                                                {Metric.Lable}, 'Zeit [s]', 'Iterationen');   
        xlim([min(TimestampsDuration,[],'all'), max(TimestampsDuration,[],'all')]);
        ylim([0 MaximumIterations]);
        legend('Location','northwest')
        
        % individual area plots
        for m = 1:M
            DurationTotal = Metric(m).DurationTotal;
            DurationSolver = Metric(m).DurationSolver;
            DurationCovariance = Metric(m).DurationCovariance;
            DurationMarginal = Metric(m).DurationMarginal;
            DurationAdaptive = Metric(m).DurationAdaptive;
            DurationOverhead = DurationTotal - (DurationSolver + DurationCovariance + DurationMarginal + DurationAdaptive);
            
            DurrationArray =[DurationOverhead,...
                             DurationAdaptive,...
                             DurationMarginal,...
                             DurationCovariance,...                             
                             DurationSolver]*1000; % convert to ms

            LableDuration = {'Overhead',...
                             'Adaptive Error Model',...
                             'Marginalization',...
                             'Covariance Recovery',...
                             'Solver'};
            
            TimestampsArray = repmat(TimestampsDuration(:,m), [1 size(DurrationArray,2)]);
                         
            Handles{end+1} = plot.createAreaPlotGeneric(TimestampsArray, DurrationArray, LableDuration, 'Time [s]', 'Duration [ms]');
            
            % formate
            ylim([0 MaximumDuration]);
            title(Lables{m});
            legend('Location','northwest')
        end
        
    end
end

%% covariance <-> Error plot
if isfield(Metric, 'Cov')
    
    if ~isempty(Metric(1).Cov)
        % prepare Data
        for m = M:-1:1
            for idx = 1:size(Metric(m).Cov,1)
                GeneralizedCovarince(m,idx) = det(squeeze(Metric(m).Cov(idx,:,:)));
            end
        end
        
        %plot
        Handles{end+1} = plot.createPlotGeneric(Timestamps', sqrt(GeneralizedCovarince), {Metric.Lable},...
                                                'Time [s]','Generalized StdDev [m]');
        set(gca, 'YScale', 'log');
    end
    
end

%% animated plot --> GIF
if Animation == true
    % config
    Filename = 'plots/AnimatedTrajectory.gif';
    Boarder = 10;
    Framerate = 60;
    StepSize = 3;
    WindowLength = 100;
    
    % set up figure
    hFig = figure;
    hAxe = axes;
    
    % find limits
    XMax = max(max(GT(:,1)), max(Trajectory(:,:,1),[],'all')) + Boarder;
    XMin = min(min(GT(:,1)), min(Trajectory(:,:,1),[],'all')) - Boarder;
    YMax = max(max(GT(:,2)), max(Trajectory(:,:,2),[],'all')) + Boarder;
    YMin = min(min(GT(:,2)), min(Trajectory(:,:,2),[],'all')) - Boarder;
    
    
    % plot GT first
    hPlotGT = plot(hAxe, GT(:,1),GT(:,2),'LineWidth',PlotConfig.Line.Width,'Color',[0.0 0.0 0.0]);
    hPlotGT.DisplayName = 'Ground Truth';
    xlabel('East [m]');
    ylabel('North [m]');
    
    for idx = 1:floor(size(Trajectory,2)/StepSize)
        
        hold on
        % plot trajectories
        for m = 1:M
            hPlotPints{idx,m} = plot(hAxe, Trajectory(m,idx*StepSize,1),Trajectory(m,idx*StepSize,2),'.','LineWidth',PlotConfig.Line.Width,'MarkerSize',15, 'Color',PlotConfig.Color.Default(m,:));
            hPlotPints{idx,m}.DisplayName = Metric(m).Lable;
        end
        
        % format plot
        axis tight manual
        box on
        
        % set correct size
        xlim(hAxe,[XMin XMax]);
        ylim(hAxe,[YMin YMax]);
        
        % create a new legend
        legend(hAxe, 'off');
        legend(hAxe, 'Ground Truth', Metric.Lable, 'Location','northeastoutside','NumColumns',1);
        
        % draw
        drawnow
        
        % format
        plot.formatPlot(hFig, plot.loadPlotConfig([900 500]))
        hold off
        
        % Capture the plot as an image
        frame = getframe(hFig);
        im = frame2im(frame);
        
        % Write to the GIF File
        if idx == 1
            [imind, cm] = rgb2ind(im, 0.25, 'nodither');
            imwrite(imind,cm,Filename,'gif', 'Loopcount',inf, 'DelayTime', 1.0/Framerate);
        else
            [imind, ~] = rgb2ind(im, cm, 'nodither');
            imwrite(imind,cm,Filename,'gif','WriteMode','append', 'DelayTime', 1.0/Framerate);
        end
        
        % delete old lines
        for m = 1:M
            if idx > WindowLength
                delete(hPlotPints{idx-WindowLength, m});
            end
        end
    end
end


end