function [YamlOut, Labels] = translateToYaml(Config)
%TRANSLATETOYAML Summary of this function goes here
%   Detailed explanation goes here

Labels = {};
YamlOut = {};

for nError = 1:numel(Config.ErrorModel)
    for nSensorConfig = 1:numel(Config.SensorConfig)
        for nSolution = 1:numel(Config.Solution)
            
            %% select the current setup
            ErrorModel = Config.ErrorModel{nError};
            SensorConfig = Config.SensorConfig{nSensorConfig};
            Solution = Config.Solution{nSolution};
            
            %% create lable
            Labels{end+1} = '';
            for nSensor = 1:numel(SensorConfig)
                Sensor = SensorConfig{nSensor};
                
                if nSensor > 1 && ~strcmp(Sensor, 'cced') && ~strcmp(Sensor, 'cce')
                    Labels{end} = [Labels{end} ' + '];
                end
                
                switch Sensor
                    case 'odom'
                        Labels{end} = [Labels{end} 'Odom'];
                    case 'odom_ecef'
                        Labels{end} = [Labels{end} 'Odom'];
                    case 'odom2'
                        Labels{end} = [Labels{end} 'Odom 2D'];
                    case 'odom4'
                        Labels{end} = [Labels{end} 'Odom'];
                    case 'odom6'
                        Labels{end} = [Labels{end} 'Odom'];
                    case 'odom_int'
                        Labels{end} = [Labels{end} 'Odom Int.'];
                    case 'odom_radar'
                        Labels{end} = [Labels{end} 'Radar-Odom'];
                    case 'odom4_radar'
                        Labels{end} = [Labels{end} 'Radar-Odom'];
                    case 'odom_lidar'
                        Labels{end} = [Labels{end} 'Lidar-Odom'];
                    case 'imu'
                        Labels{end} = [Labels{end} 'IMU'];
                    case 'imu_pre'
                        Labels{end} = [Labels{end} 'IMU Pre-Int.'];
                    case 'uwb'
                        Labels{end} = [Labels{end} 'UWB'];
                    case 'range'
                        Labels{end} = [Labels{end} 'Ranging'];
                    case 'range_lm'
                        Labels{end} = [Labels{end} 'Ranging'];
                    case 'gnss'
                        Labels{end} = [Labels{end} 'GNSS'];
                    case 'gnss_ecef'
                        Labels{end} = [Labels{end} 'GNSS'];
                    case 'gnss_bias'
                        Labels{end} = [Labels{end} 'GNSS'];                        
                    case 'cced'
                        %do nothing
                    case 'cce'
                        %do nothing
                    case 'loop'
                        Labels{end} = [Labels{end} 'Loop Closure 2D'];
                    case 'radar'
                        Labels{end} = [Labels{end} 'Radar Registration 2D'];
                    case 'ground'
                        Labels{end} = [Labels{end} 'Ground Prior'];
                    case 'pressure'
                        Labels{end} = [Labels{end} 'Pressure'];
                    otherwise
                        Labels{end} = [Labels{end} Sensor];
                        warning(['No Label for sensor: ' Sensor]);
                end
            end
            Labels{end} = [Labels{end} ' + ' ErrorModel ' + ' Config.Solution{nSolution}];
            
            
            %% select default yaml file
            if isfield(Config, 'YAMLFile')
                DefaultFile = Config.YAMLFile;
            else
                switch Config.DatasetName
                    case 'Chemnitz'
                        DefaultFile = 'Default_Chemnitz';
                        
                    case {'Berlin_Potsdamer_Platz',...
                          'Berlin_Gendarmenmarkt',...
                          'Frankfurt_Main_Tower',...
                          'Frankfurt_Westend_Tower',...
                          'Berlin_Potsdamer_Platz_RTK',...
                          'Berlin_Gendarmenmarkt_RTK',...
                          'Frankfurt_Main_Tower_RTK',...
                          'Frankfurt_Westend_Tower_RTK'}
                        DefaultFile = 'Default_smartLoc';
                        
                    case {'Hongkong_Loop_Small', 'Hongkong_Loop_Large',...
                          'UrbanNav_HK_1','UrbanNav_HK_2',...
                          'UrbanNav_HK_Medium_Urban','UrbanNav_HK_Deep_Urban','UrbanNav_HK_Harsh_Urban',...
                          'UrbanNav_TK_Shinjuku','UrbanNav_TK_Obaida'}
                        DefaultFile = 'Default_HK';
                        
                    case {'Radar_Synth', 'Radar_Synth_Cluster', 'Radar_Synth_Cluster_Far', 'Radar_Synth_Outlier', 'Radar_Synth_Cluster_Outlier', 'Radar_Synth_Outrange_Outlier', 'Radar_Turmbau_1', 'Radar_Synth_RAL', 'Radar_Synth_RAL_100'}
                        DefaultFile = 'Default_Radar';
                        
                    otherwise
                        DefaultFile = 'Default';
                end
            end
            
            %% translate the config using the actual YAML file
            % create path
            RelativePath = fileparts(mfilename('fullpath'));
            DefaultPath = [RelativePath filesep '..' filesep '..' filesep 'config' filesep DefaultFile '.yaml'];
            
            % read default file
            YamlDefault = yaml.ReadYaml(DefaultPath,1,1);
            
            YamlOut{end+1} = struct;
            
            % solution
            try
                YamlOut{end}.config.solution =  YamlDefault.solutions.(Solution);
            catch
                error(['Solution type ' Solution ' does not exist in ' DefaultFile '!']);
            end
            YamlOut{end}.config.solution.estimate_cov = Config.EstimateCov;
            
            
            % factor graph
            if Config.IsSync
                if (strcmp(Config.DatasetName,'Turmbau_2') || strcmp(Config.DatasetName,'Turmbau_1'))
                    GraphType = 'sync_uwb';
                else
                    GraphType = 'sync';
                end
            else
                GraphType = 'async';
            end
            
            try
                YamlOut{end}.config.graph =  YamlDefault.architecture.(GraphType);
            catch
                error(['Graph type ' GraphType ' does not exist in ' DefaultFile '!']);
            end
            
            % sensors + error model
            YamlOut{end}.config.factors = {};
            for nSensor = 1:numel(SensorConfig)

                % select sensor
                Sensor = SensorConfig{nSensor};
                try
                    YamlOut{end}.config.factors{end+1} = YamlDefault.factors.(Sensor);
                catch
                    error(['Factor type ' Sensor ' does not exist in ' DefaultFile '!']);
                end
                
                % overwrite with selected error model
                if strcmp(Sensor, 'gnss') ||...
                    strcmp(Sensor, 'gnss_ecef') ||...
                    strcmp(Sensor, 'gnss_bias') ||...
                    strcmp(Sensor, 'uwb') ||...
                    strcmp(Sensor, 'loop') ||...
                    strcmp(Sensor, 'range') ||...
                    strcmp(Sensor, 'range_lm') ||...
                    strcmp(Sensor, 'radar')
                    try
                        YamlOut{end}.config.factors{end}.error = YamlDefault.errors.(ErrorModel);
                    catch
                        error(['Error model ' ErrorModel ' does not exist in ' DefaultFile '!']);
                    end
                end

                % manipulate loop closure parameter
                if strcmp(Sensor, 'loop') && isfield(Config, 'Loop')
                    if isfield(Config.Loop, 'Threshold')
                        YamlOut{end}.config.factors{nSensor}.threshold = Config.Loop.Threshold;
                    end
                end
                
            end
        end
    end
end

end

