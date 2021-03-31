function [] = writeYamlReference(SensorData, OutputPath)
%WRITEYAMLREFERENCE Summary of this function goes here
%   Detailed explanation goes here

% add absolute time 
if isfield(SensorData, 'StartTime')
    YAML.StartTime = SensorData.StartTime;
else
    YAML.StartTime = 0;
end

% add spatial frame
if isfield(SensorData, 'GT_Position3')
    if isfield(SensorData.GT_Position3, 'Frame')
        YAML.Frame = SensorData.GT_Position3.Frame;
    end
    if isfield(SensorData.GT_Position3, 'ReferencePoint')
        YAML.Origin = SensorData.GT_Position3.ReferencePoint;
    end
end

%% Write result
if exist(['+yaml' filesep 'ReadYaml.m'],'file') 
    yaml.WriteYaml(OutputPath, YAML, 0);
end

end

