function [] = writeYamlConfig(Config, InputPath, OutputPath)

%% read default
YAML_in = yaml.ReadYaml(InputPath,1,1);

%% parse external config
% solution
switch Config.Solution.Type
    case 'batch'
        YAML_out.solution = YAML_in.solutions.batch;
    case 'window'
        YAML_out.solution = YAML_in.solutions.sliding_window;
    case 'filter'
        YAML_out.solution = YAML_in.solutions.filter;
    case 'smoother'
        YAML_out.solution = YAML_in.solutions.smoother;
    case 'none'
        YAML_out.solution = YAML_in.solutions.none;
    otherwise
        error(['Wrong solution name: ' Config.Solution.Type]);
end
YAML_out.solution.estimate_cov = Config.Solution.EstimateCov;

% architecture
switch Config.Graph.Type
    case 'sync_gnss'
        YAML_out.graph = YAML_in.architecture.sync_gnss;
    case 'sync_uwb'
        YAML_out.graph = YAML_in.architecture.sync_uwb;
    case 'sync'
        YAML_out.graph = YAML_in.architecture.sync;
    case 'async'
        YAML_out.graph = YAML_in.architecture.async;
    otherwise
        error(['Wrong architecture name: ' Config.Graph.Type]);
end

% factors
YAML_out.factors = {};
for nFactor = 1:numel(Config.Sensors.List)
    
    % check if factor config exists
    if isfield(YAML_in.factors, Config.Sensors.List{nFactor}) == false
        error(['Wrong factor name: ' Config.Sensors.List{nFactor}]);        
    end
    
    % copy config
    YAML_out.factors{end+1} = YAML_in.factors.(Config.Sensors.List{nFactor});
    
    % chage error model to robust one
    if strcmp(Config.Sensors.List{nFactor}, 'gnss') ||...
       strcmp(Config.Sensors.List{nFactor}, 'uwb') ||...
       strcmp(Config.Sensors.List{nFactor}, 'loop') ||...
       strcmp(Config.Sensors.List{nFactor}, 'range') ||...
       strcmp(Config.Sensors.List{nFactor}, 'radar')
       
        YAML_out.factors{end}.error = YAML_in.errors.(Config.ErrorModel);
    end

end

%% Write result
yaml.WriteYaml(OutputPath, YAML_out, 0);
