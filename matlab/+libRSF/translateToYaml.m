function [YAMLs, Lables] = translateToYaml(Config)
%TRANSLATETOYAML Summary of this function goes here
%   Detailed explanation goes here

Lables = {};
YAMLs = {};

for m = 1:numel(Config.ErrorModel)
    for n = 1:numel(Config.SensorConfig)
        for k =1:numel(Config.Solution)
            YAML = [];
            
            if isfield(Config, 'YAMLFile')
                YAML.Default = Config.YAMLFile;
            else
                % choose default yaml options
                switch Config.DatasetName
                    case 'Chemnitz'
                        YAML.Default = 'Default_Chemnitz';
                        Config.IsSync = true;
                        
                    case {'Berlin_Potsdamer_Platz', 'Berlin_Gendarmenmarkt',  'Frankfurt_Main_Tower', 'Frankfurt_Westend_Tower'}
                        YAML.Default = 'Default_smartLoc';
                        Config.IsSync = true;
                        
                    case {'Hongkong_Loop_Small', 'Hongkong_Loop_Large'}
                        YAML.Default = 'Default_HK';
                        Config.IsSync = true;
                        
                    case {'Radar_Synth', 'Radar_Synth_Cluster', 'Radar_Synth_Cluster_Far', 'Radar_Synth_Outlier', 'Radar_Synth_Cluster_Outlier', 'Radar_Synth_Outrange_Outlier', 'Radar_Turmbau_1', 'Radar_Synth_RAL', 'Radar_Synth_RAL_100'}
                        YAML.Default = 'Default_Radar';
                        
                    otherwise
                        YAML.Default = 'Default';
                end
            end

            % solution setup
            YAML.Solution.Type = Config.Solution{k};

            % cov est
            YAML.Solution.EstimateCov = Config.EstimateCov;

            % set graph architecture
            if Config.IsSync
                if (strcmp(Config.DatasetName,'Turmbau_2') || strcmp(Config.DatasetName,'Turmbau_1'))
                    YAML.Graph.Type = 'sync_uwb';    
                else
                    YAML.Graph.Type = 'sync';                    
                end
            else
                YAML.Graph.Type = 'async';
            end

            % sensor setup
            YAML.Sensors.List = Config.SensorConfig{n};
            Lables{end+1} = '';
            for i = 1:numel(Config.SensorConfig{n})
                if i > 1
                    Lables{end} = [Lables{end} ' + '];
                end
                
                switch Config.SensorConfig{n}{i}
                    case 'odom'
                        Lables{end} = [Lables{end} 'Odom'];
                    case 'odom2'
                        Lables{end} = [Lables{end} 'Odom 2D'];
                    case 'odom_int'
                        Lables{end} = [Lables{end} 'Odom Int.'];
                    case 'odom_radar'
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
                    otherwise
                    error(['Wrong sensor config: ' Config.SensorConfig{n}{i}]);
                end
            end

            % error setup
            YAML.ErrorModel = Config.ErrorModel{m};
            Lables{end} = [Lables{end} ' + ' Config.ErrorModel{m} ' + ' Config.Solution{k}];

            % store config
            YAMLs{end+1} = YAML;
        end
    end
end

end

