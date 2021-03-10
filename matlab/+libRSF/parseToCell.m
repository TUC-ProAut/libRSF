function [MeasurementCell, GTCell] = parseToCell(Data)
%PARSETOCELL Convert structs with measurements to an unified cell format (preprocessing for csv export)

MaxCellWidth = 25;

%% initial conditions
Pos3Cell = cell(0,MaxCellWidth);
if isfield(Data,'Position3')
    [Pos3Cell{1:length(Data.Position3.Time),1}]    = deal('point3');
    Pos3Cell(:,2)                          = num2cell(Data.Position3.Time);
    Pos3Cell(:,3)                          = num2cell(Data.Position3.X);
    Pos3Cell(:,4)                          = num2cell(Data.Position3.Y);
    Pos3Cell(:,5)                          = num2cell(Data.Position3.Z);
    Pos3Cell(:,6:14)                       = num2cell(Data.Position3.Cov);
end

Pos2Cell = cell(0,MaxCellWidth);
if isfield(Data,'Position2')
    [Pos2Cell{1:length(Data.Position2.Time),1}]    = deal('point2');
    Pos2Cell(:,2)                          = num2cell(Data.Position2.Time);
    Pos2Cell(:,3)                          = num2cell(Data.Position2.X);
    Pos2Cell(:,4)                          = num2cell(Data.Position2.Y);
    Pos2Cell(:,5:8)                        = num2cell(reshape(Data.Position2.Cov,[],4));
end

PosID2Cell = cell(0,MaxCellWidth);
if isfield(Data,'PositionID2')
    [PosID2Cell{1:length(Data.PositionID2.Time),1}]    = deal('point_id2');
    PosID2Cell(:,2)                          = num2cell(Data.PositionID2.Time);
    PosID2Cell(:,3)                          = num2cell(Data.PositionID2.X);
    PosID2Cell(:,4)                          = num2cell(Data.PositionID2.Y);
    PosID2Cell(:,5)                          = num2cell(Data.PositionID2.ID);
    PosID2Cell(:,6)                          = num2cell(Data.PositionID2.Idx);
    PosID2Cell(:,7)                          = num2cell(Data.PositionID2.Conf);
    PosID2Cell(:,8:11)                       = num2cell(Data.PositionID2.Cov);
end

PosID3Cell = cell(0,MaxCellWidth);
if isfield(Data,'PositionID3')
    [PosID3Cell{1:length(Data.PositionID3.Time),1}]    = deal('point_id3');
    PosID3Cell(:,2)                          = num2cell(Data.PositionID3.Time);
    PosID3Cell(:,3)                          = num2cell(Data.PositionID3.X);
    PosID3Cell(:,4)                          = num2cell(Data.PositionID3.Y);
    PosID3Cell(:,5)                          = num2cell(Data.PositionID3.Z);
    PosID3Cell(:,6)                          = num2cell(Data.PositionID3.Vx);
    PosID3Cell(:,7)                          = num2cell(Data.PositionID3.Vy);
    PosID3Cell(:,8)                          = num2cell(Data.PositionID3.Vz);
    PosID3Cell(:,9)                          = num2cell(Data.PositionID3.ID);
    PosID3Cell(:,10)                         = num2cell(Data.PositionID3.Idx);
    PosID3Cell(:,11)                         = num2cell(Data.PositionID3.Conf);
    PosID3Cell(:,12:20)                      = num2cell(Data.PositionID3.Cov);
    PosID3Cell(:,21:23)                      = num2cell(Data.PositionID3.WLH);
    PosID3Cell(:,24)                         = num2cell(Data.PositionID3.R);
    PosID3Cell(:,25:28)                      = num2cell(Data.PositionID3.R_Quat);
    PosID3Cell(:,29)                         = num2cell(Data.PositionID3.ClassNum);
    PosID3Cell(:,30)                         = num2cell(Data.PositionID3.KeyNum);
end

QuatCell = cell(0,MaxCellWidth);
if isfield(Data,'Quaternion')
    [QuatCell{1:length(Data.Quaternion.Time),1}]    = deal('quaternion');
    QuatCell(:,2)                                 = num2cell(Data.Quaternion.Time);
    QuatCell(:,3)                                 = num2cell(Data.Quaternion.X);
    QuatCell(:,4)                                 = num2cell(Data.Quaternion.Y);
    QuatCell(:,5)                                 = num2cell(Data.Quaternion.Z);
    QuatCell(:,6)                                 = num2cell(Data.Quaternion.W);
    QuatCell(:,7:22)                              = num2cell(zeros(length(Data.Quaternion.Time),16));
end
Speed3Cell = cell(0,MaxCellWidth);
if isfield(Data,'SpeedBias')
    [Speed3Cell{1:length(Data.Position.Time),1}]    = deal('imubias');
    Speed3Cell(:,2)                          = num2cell(Data.SpeedBias.Time);
    Speed3Cell(:,3)                          = num2cell(Data.SpeedBias.vx);
    Speed3Cell(:,4)                          = num2cell(Data.SpeedBias.vy);
    Speed3Cell(:,5)                          = num2cell(Data.SpeedBias.vz);
    Speed3Cell(:,6)                          = num2cell(Data.SpeedBias.bax);
    Speed3Cell(:,7)                          = num2cell(Data.SpeedBias.bay);
    Speed3Cell(:,8)                          = num2cell(Data.SpeedBias.baz);
    Speed3Cell(:,9)                          = num2cell(Data.SpeedBias.btrx);
    Speed3Cell(:,10)                          = num2cell(Data.SpeedBias.btry);
    Speed3Cell(:,11)                          = num2cell(Data.SpeedBias.btrz);
end

%% IMU
IMUCell = cell(0,MaxCellWidth);
if isfield(Data,'IMU')
    [IMUCell{1:length(Data.IMU.Time),1}]    = deal('imu');
    IMUCell(:,2)                            = num2cell(Data.IMU.Time);
    IMUCell(:,3:5)                          = num2cell(Data.IMU.Acc);
    IMUCell(:,6:8)                          = num2cell(Data.IMU.TR);
    IMUCell(:,9:11)                         = num2cell(Data.IMU.AccCov);
    IMUCell(:,12:14)                        = num2cell(Data.IMU.TRCov);
end

%% Odometry
Odom2Cell = cell(0,MaxCellWidth);
if isfield(Data,'Odom2')
    [Odom2Cell{1:length(Data.Odom2.Time),1}]    = deal('odom2');
    Odom2Cell(:,2)                              = num2cell(Data.Odom2.Time);
    Odom2Cell(:,3:4)                            = num2cell(Data.Odom2.Vel);
    Odom2Cell(:,5)                              = num2cell(Data.Odom2.TR);
    Odom2Cell(:,6:7)                            = num2cell(Data.Odom2.VelCov);
    Odom2Cell(:,8)                              = num2cell(Data.Odom2.TRCov);
end

Odom2DiffCell = cell(0,MaxCellWidth);

Odom3Cell = cell(0,MaxCellWidth);
if isfield(Data,'Odom3')
    [Odom3Cell{1:length(Data.Odom3.Time),1}]  = deal('odom3');
    Odom3Cell(:,2)                            = num2cell(Data.Odom3.Time);
    Odom3Cell(:,3:5)                          = num2cell(Data.Odom3.Vel);
    Odom3Cell(:,6:8)                          = num2cell(Data.Odom3.TR);
    Odom3Cell(:,9:11)                         = num2cell(Data.Odom3.VelCov);
    Odom3Cell(:,12:14)                        = num2cell(Data.Odom3.TRCov);
end

%% odometry from different sensors
Odom3RadarCell = cell(0,MaxCellWidth);
if isfield(Data,'Odom3Radar')
    [Odom3RadarCell{1:length(Data.Odom3Radar.Time),1}]  = deal('odom3radar');
    Odom3RadarCell(:,2)                            = num2cell(Data.Odom3Radar.Time);
    Odom3RadarCell(:,3:5)                          = num2cell(Data.Odom3Radar.Vel);
    Odom3RadarCell(:,6:8)                          = num2cell(Data.Odom3Radar.TR);
    Odom3RadarCell(:,9:11)                         = num2cell(Data.Odom3Radar.VelCov);
    Odom3RadarCell(:,12:14)                        = num2cell(Data.Odom3Radar.TRCov);
end

Odom3LaserCell = cell(0,MaxCellWidth);
if isfield(Data,'Odom3Laser')
    [Odom3LaserCell{1:length(Data.Odom3Laser.Time),1}]  = deal('odom3laser');
    Odom3LaserCell(:,2)                            = num2cell(Data.Odom3Laser.Time);
    Odom3LaserCell(:,3:5)                          = num2cell(Data.Odom3Laser.Vel);
    Odom3LaserCell(:,6:8)                          = num2cell(Data.Odom3Laser.TR);
    Odom3LaserCell(:,9:11)                         = num2cell(Data.Odom3Laser.VelCov);
    Odom3LaserCell(:,12:14)                        = num2cell(Data.Odom3Laser.TRCov);
end

Odom3VIOCell = cell(0,MaxCellWidth);
if isfield(Data,'Odom3VIO')
    [Odom3VIOCell{1:length(Data.Odom3VIO.Time),1}]  = deal('odom3vio');
    Odom3VIOCell(:,2)                            = num2cell(Data.Odom3VIO.Time);
    Odom3VIOCell(:,3:5)                          = num2cell(Data.Odom3VIO.Vel);
    Odom3VIOCell(:,6:8)                          = num2cell(Data.Odom3VIO.TR);
    Odom3VIOCell(:,9:11)                         = num2cell(Data.Odom3VIO.VelCov);
    Odom3VIOCell(:,12:14)                        = num2cell(Data.Odom3VIO.TRCov);
end

%% Ranging
Range2Cell = cell(0,MaxCellWidth);
if isfield(Data,'Range2')
    [Range2Cell{1:length(Data.Range2.Time),1}]    = deal('range2');
    Range2Cell(:,2)                          = num2cell(Data.Range2.Time);
    Range2Cell(:,3)                          = num2cell(Data.Range2.Mean);
    Range2Cell(:,4)                          = num2cell(Data.Range2.Cov);
    Range2Cell(:,5:6)                        = num2cell(Data.Range2.ModulePos);
    Range2Cell(:,7)                          = num2cell(Data.Range2.ModuleID);
    Range2Cell(:,8)                          = num2cell(Data.Range2.SNR);
end

Range3Cell = cell(0,MaxCellWidth);
if isfield(Data,'Range3')
    [Range3Cell{1:length(Data.Range3.Time),1}]    = deal('range3');
    Range3Cell(:,2)                          = num2cell(Data.Range3.Time);
    Range3Cell(:,3)                          = num2cell(Data.Range3.Mean);
    Range3Cell(:,4)                          = num2cell(Data.Range3.Cov);
    Range3Cell(:,5:7)                        = num2cell(Data.Range3.ModulePos);
    Range3Cell(:,8)                          = num2cell(Data.Range3.ModuleID);
    Range3Cell(:,9)                          = num2cell(Data.Range3.SNR);
end

%% GNSS
Pseudorange2Cell = cell(0,MaxCellWidth);

Pseudorange3Cell = cell(0,MaxCellWidth);
if isfield(Data,'Pseudorange3')
    [Pseudorange3Cell{1:length(Data.Pseudorange3.Time),1}]    = deal('pseudorange3');
    Pseudorange3Cell(:,2)                          = num2cell(Data.Pseudorange3.Time);
    Pseudorange3Cell(:,3)                          = num2cell(Data.Pseudorange3.Mean);
    Pseudorange3Cell(:,4)                          = num2cell(Data.Pseudorange3.Cov);
    Pseudorange3Cell(:,5:7)                        = num2cell(Data.Pseudorange3.SatPos);
    Pseudorange3Cell(:,8)                          = num2cell(Data.Pseudorange3.SatID);
    Pseudorange3Cell(:,9)                          = num2cell(Data.Pseudorange3.SatElevation);
    Pseudorange3Cell(:,10)                         = num2cell(Data.Pseudorange3.SNR);
end

%% raw Laser

%% raw Radar

%% raw Camera

%% air pressuere
PressureCell = cell(0,MaxCellWidth);
if isfield(Data,'Pressure')
    [PressureCell{1:length(Data.Pressure.Time),1}]    = deal('pressure');
    PressureCell(:,2)                          = num2cell(Data.Pressure.Time);
    PressureCell(:,3)                          = num2cell(Data.Pressure.Mean);
    PressureCell(:,4)                          = num2cell(Data.Pressure.Cov);
end

%% Place recognition
LoopCell = cell(0,MaxCellWidth);
if isfield(Data,'Loop')
    [LoopCell{1:length(Data.Loop.Time),1}]    = deal('loop');
    LoopCell(:,2)                          = num2cell(Data.Loop.Time);
    LoopCell(:,3)                          = num2cell(Data.Loop.TimeRef);
end


%% GT
GT2Cell = cell(0,MaxCellWidth);
if isfield(Data,'GT_Position2')
    [GT2Cell{1:length(Data.GT_Position2.Time),1}]    = deal('point2');
    GT2Cell(:,2)                            = num2cell(Data.GT_Position2.Time);
    GT2Cell(:,3)                            = num2cell(Data.GT_Position2.X);
    GT2Cell(:,4)                            = num2cell(Data.GT_Position2.Y);
    GT2Cell(:,5:8)                         = num2cell(zeros(numel(Data.GT_Position2.Time),4));
end

GT3Cell = cell(0,MaxCellWidth);
if isfield(Data,'GT_Position3')
    [GT3Cell{1:length(Data.GT_Position3.Time),1}]    = deal('point3');
    GT3Cell(:,2)                            = num2cell(Data.GT_Position3.Time);
    GT3Cell(:,3)                            = num2cell(Data.GT_Position3.X);
    GT3Cell(:,4)                            = num2cell(Data.GT_Position3.Y);
    GT3Cell(:,5)                            = num2cell(Data.GT_Position3.Z);
    GT3Cell(:,6:14)                         = num2cell(zeros(numel(Data.GT_Position3.Time),9));
end

%% merge data
MeasurementCell = [ Pos3Cell;Pos2Cell; PosID2Cell; PosID3Cell; QuatCell; Speed3Cell;...
                    IMUCell;...
                    PressureCell;...
                    Odom2Cell; Odom2DiffCell; Odom3Cell;...
                    Odom3RadarCell; Odom3LaserCell; Odom3VIOCell;...
                    Range2Cell; Range3Cell;...
                    Pseudorange2Cell; Pseudorange3Cell;...
                    LoopCell];
                
GTCell = [GT2Cell; GT3Cell];

end

