function [YamlOut, Lables] = translateToYaml(Config)
%TRANSLATETOYAML Summary of this function goes here
%   Detailed explanation goes here

Lables = {};
YamlOut = {};

for nError = 1:numel(Config.ErrorModel)
    for nSensorConfig = 1:numel(Config.SensorConfig)
        for nSolution = 1:numel(Config.Solution)
            
            %% select the current setup
            ErrorModel = Config.ErrorModel{nError};
            SensorConfig = Config.SensorConfig{nSensorConfig};
            Solution = Config.Solution{nSolution};
            
            %% create lable
            Lables{end+1} = '';
            for nSensor = 1:numel(SensorConfig)
                Sensor = SensorConfig{nSensor};
                
                if nSensor > 1 && ~strcmp(Sensor, 'cced') && ~strcmp(Sensor, 'cce')
                    Lables{end} = [Lables{end} ' + '];
                end
                
                switch Sensor
                    case 'odom'
                        Lables{end} = [Lables{end} 'Odom'];
                    case 'odom_ecef'
                        Lables{end} = [Lables{end} 'Odom'];
                    case 'odom2'
                        Lables{end} = [Lables{end} 'Odom 2D'];
                    case 'odom4'
                        Lables{end} = [Lables{end} 'Odom'];
                    case 'odom6'
                        Lables{end} = [Lables{end} 'Odom'];
                    case 'odom_int'
                        Lables{end} = [Lables{end} 'Odom Int.'];
                    case 'odom_radar'
                        Lables{end} = [Lables{end} 'Radar-Odom'];
                    case 'odom4_radar'
                        Lables{end} = [Lables{end} 'Radar-Odom'];
                    case 'odom_lidar'
                        Lables{end} = [Lables{end} 'Lidar-Odom'];
                    case 'imu'
                        Lables{end} = [Lables{end} 'IMU'];
                    case 'imu_pre'
                        Lables{end} = [Lables{end} 'IMU Pre-Int.'];
                    case 'uwb'
                        Lables{end} = [Lables{end} 'UWB'];
                    case 'range'
                        Lables{end} = [Lables{end} 'Ranging'];
                    case 'gnss'
                        Lables{end} = [Lables{end} 'GNSS'];
                    case 'gnss_ecef'
                        Lables{end} = [Lables{end} 'GNSS'];
                    case 'cced'
                        %do nothing
                    case 'cce'
                        %do nothing
                    case 'loop'
                        Lables{end} = [Lables{end} 'Loop Closure 2D'];
                    case 'radar'
                        Lables{end} = [Lables{end} 'Radar Registration 2D'];
                    case 'ground'
                        Lables{end} = [Lables{end} 'Ground Prior'];
                    case 'pressure'
                        Lables{end} = [Lables{end} 'Pressure'];
                    otherwise
                        Lables{end} = [Lables{end} Sensor];
                        warning(['No Lable for sensor: ' Sensor]);
                end
            end
            Lables{end} = [Lables{end} ' + ' ErrorModel ' + ' Config.Solution{nSolution}];
            
            
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
                        
                    case {'Hongkong_Loop_Small', 'Hongkong_Loop_Large'}
                        DefaultFile = 'Default_HK';
                        
                    case {'Radar_Synth', 'Radar_Synth_Cluster', 'Radar_Synth_Cluster_Far', 'Radar_Synth_Outlier', 'Radar_Synth_Cluster_Outlier', 'Radar_Synth_Outrange_Outlier', 'Radar_Turmbau_1', 'Radar_Synth_RAL', 'Radar_Synth_RAL_100'}
                        DefaultFile = 'Default_Radar';
                        
                    otherwise
                        DefaultFile = 'Default';
                end
            end
            
            %% overwrite default options
            if strcmp(Config.DatasetName, 'Chemnitz') || ...
                    strcmp(Config.DatasetName, 'Berlin_Potsdamer_Platz') || ...
                    strcmp(Config.DatasetName, 'Berlin_Gendarmenmarkt') || ...
                    strcmp(Config.DatasetName, 'Frankfurt_Main_Tower') || ...
                    strcmp(Config.DatasetName, 'Frankfurt_Westend_Tower') || ...
                    strcmp(Config.DatasetName, 'Berlin_Potsdamer_Platz_RTK') || ...
                    strcmp(Config.DatasetName, 'Berlin_Gendarmenmarkt_RTK') || ...
                    strcmp(Config.DatasetName, 'Frankfurt_Main_Tower_RTK') || ...
                    strcmp(Config.DatasetName, 'Frankfurt_Westend_Tower_RTK') || ...
                    strcmp(Config.DatasetName, 'Hongkong_Loop_Small') || ...
                    strcmp(Config.DatasetName, 'Hongkong_Loop_Large')
                
                Config.IsSync = true;
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
                    strcmp(Sensor, 'uwb') ||...
                    strcmp(Sensor, 'loop') ||...
                    strcmp(Sensor, 'range') ||...
                    strcmp(Sensor, 'radar')
                    try
                        YamlOut{end}.config.factors{end}.error = YamlDefault.errors.(ErrorModel);
                    catch
                        error(['Error model ' ErrorModel ' does not exist in ' DefaultFile '!']);
                    end
                end
                
            end
        end
    end
end

end

