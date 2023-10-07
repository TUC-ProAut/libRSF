function [MeasurementCell, GTCell] = parseToCell(Data)
%PARSETOCELL Convert structs with measurements to an unified cell format (preprocessing for csv export)

MaxCellWidth = 30;

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
    PosID2Cell(:,6:9)                        = num2cell(Data.PositionID2.Cov);
end

PosConfID2Cell = cell(0,MaxCellWidth);
if isfield(Data,'PositionConfID2')
    [PosConfID2Cell{1:length(Data.PositionConfID2.Time),1}]    = deal('point_conf_id2');
    PosConfID2Cell(:,2)                          = num2cell(Data.PositionConfID2.Time);
    PosConfID2Cell(:,3)                          = num2cell(Data.PositionConfID2.X);
    PosConfID2Cell(:,4)                          = num2cell(Data.PositionConfID2.Y);
    PosConfID2Cell(:,5)                          = num2cell(Data.PositionConfID2.ID);
    PosConfID2Cell(:,6)                          = num2cell(Data.PositionConfID2.Idx);
    PosConfID2Cell(:,7)                          = num2cell(Data.PositionConfID2.Conf);
    PosConfID2Cell(:,8:11)                       = num2cell(Data.PositionConfID2.Cov);
end

BoundingBox3Cell = cell(0,MaxCellWidth);
if isfield(Data,'BoundingBox3')
    [BoundingBox3Cell{1:length(Data.BoundingBox3.Time),1}]    = deal('bounding_box_3');
    BoundingBox3Cell(:,2)                          = num2cell(Data.BoundingBox3.Time);
    BoundingBox3Cell(:,3)                          = num2cell(Data.BoundingBox3.X);
    BoundingBox3Cell(:,4)                          = num2cell(Data.BoundingBox3.Y);
    BoundingBox3Cell(:,5)                          = num2cell(Data.BoundingBox3.Z);
    BoundingBox3Cell(:,6)                          = num2cell(Data.BoundingBox3.Vx);
    BoundingBox3Cell(:,7)                          = num2cell(Data.BoundingBox3.Vy);
    BoundingBox3Cell(:,8)                          = num2cell(Data.BoundingBox3.Vz);
    BoundingBox3Cell(:,9)                          = num2cell(Data.BoundingBox3.ID);
    BoundingBox3Cell(:,10)                         = num2cell(Data.BoundingBox3.Idx);
    BoundingBox3Cell(:,11)                         = num2cell(Data.BoundingBox3.Conf);
    BoundingBox3Cell(:,12:20)                      = num2cell(Data.BoundingBox3.Cov);
    BoundingBox3Cell(:,21:23)                      = num2cell(Data.BoundingBox3.WLH);
    BoundingBox3Cell(:,24)                         = num2cell(Data.BoundingBox3.R);
    BoundingBox3Cell(:,25:28)                      = num2cell(Data.BoundingBox3.R_Quat);
    BoundingBox3Cell(:,29)                         = num2cell(Data.BoundingBox3.ClassNum);
    BoundingBox3Cell(:,30)                         = num2cell(Data.BoundingBox3.KeyNum);
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
if isfield(Data,'Odom2Diff')
    error('todo');
end

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

%% relative poses
PoseBetween2Cell = cell(0,MaxCellWidth);
if isfield(Data,'PoseBetween2')
    [PoseBetween2Cell{1:length(Data.PoseBetween2.Time),1}]    = deal('pose_between2');
    PoseBetween2Cell(:,2)                              = num2cell(Data.PoseBetween2.Time);
    PoseBetween2Cell(:,3)                              = num2cell(Data.PoseBetween2.TimeRef);
    PoseBetween2Cell(:,4)                              = num2cell(Data.PoseBetween2.X);
    PoseBetween2Cell(:,5)                              = num2cell(Data.PoseBetween2.Y);
    PoseBetween2Cell(:,6)                              = num2cell(Data.PoseBetween2.Yaw);
    PoseBetween2Cell(:,7:15)                           = num2cell(Data.PoseBetween2.Cov);
end

PoseBetween3Cell = cell(0,MaxCellWidth);
if isfield(Data,'PoseBetween3')
    error('todo');
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

%% Range SLAM
RangeLM2Cell = cell(0,MaxCellWidth);
if isfield(Data,'RangeLM2')
    [RangeLM2Cell{1:length(Data.RangeLM2.Time),1}]    = deal('range_lm2');
    RangeLM2Cell(:,2)                          = num2cell(Data.RangeLM2.Time);
    RangeLM2Cell(:,3)                          = num2cell(Data.RangeLM2.Mean);
    RangeLM2Cell(:,4)                          = num2cell(Data.RangeLM2.Cov);
    RangeLM2Cell(:,5)                          = num2cell(Data.RangeLM2.ModuleID);
    RangeLM2Cell(:,6)                          = num2cell(Data.RangeLM2.SNR);
end

RangeLM3Cell = cell(0,MaxCellWidth);
if isfield(Data,'RangeLM3')
    [RangeLM3Cell{1:length(Data.RangeLM3.Time),1}]    = deal('range_lm3');
    RangeLM3Cell(:,2)                          = num2cell(Data.RangeLM3.Time);
    RangeLM3Cell(:,3)                          = num2cell(Data.RangeLM3.Mean);
    RangeLM3Cell(:,4)                          = num2cell(Data.RangeLM3.Cov);
    RangeLM3Cell(:,5)                          = num2cell(Data.RangeLM3.ModuleID);
    RangeLM3Cell(:,6)                          = num2cell(Data.RangeLM3.SNR);
end

%% GNSS
Pseudorange2Cell = cell(0,MaxCellWidth);
if isfield(Data,'Pseudorange2')
    error('todo');
end

Pseudorange3Cell = cell(0,MaxCellWidth);
if isfield(Data,'Pseudorange3')
    [Pseudorange3Cell{1:length(Data.Pseudorange3.Time),1}]    = deal('pseudorange3');
    Pseudorange3Cell(:,2)                          = num2cell(Data.Pseudorange3.Time);
    Pseudorange3Cell(:,3)                          = num2cell(Data.Pseudorange3.Mean);
    Pseudorange3Cell(:,4)                          = num2cell(Data.Pseudorange3.Cov);
    Pseudorange3Cell(:,5:7)                        = num2cell(Data.Pseudorange3.SatPos);
    Pseudorange3Cell(:,8)                          = num2cell(Data.Pseudorange3.SatID);
    Pseudorange3Cell(:,9)                          = num2cell(Data.Pseudorange3.SatSysNum);
    Pseudorange3Cell(:,10)                          = num2cell(Data.Pseudorange3.SatElevation);
    Pseudorange3Cell(:,11)                         = num2cell(Data.Pseudorange3.SNR);
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

%% Place Recognition and SLAM
LoopCell = cell(0,MaxCellWidth);
if isfield(Data,'Loop')
    [LoopCell{1:length(Data.Loop.Time),1}]    = deal('loop');
    LoopCell(:,2)                             = num2cell(Data.Loop.Time);
    LoopCell(:,3)                             = num2cell(Data.Loop.TimeRef);
    LoopCell(:,4)                             = num2cell(Data.Loop.Similarity);
end

BearingRangeID2Cell = cell(0,MaxCellWidth);
if isfield(Data,'BearingRangeID2')
    [BearingRangeID2Cell{1:length(Data.BearingRangeID2.Time),1}] = deal('bearing_range_id_2');
    BearingRangeID2Cell(:,2)        = num2cell(Data.BearingRangeID2.Time);
    BearingRangeID2Cell(:,3:4)      = num2cell(Data.BearingRangeID2.Mean);
    BearingRangeID2Cell(:,5:6)      = num2cell(Data.BearingRangeID2.Cov);
    BearingRangeID2Cell(:,7)        = num2cell(Data.BearingRangeID2.ID); 
end

%% GT
GT2Cell = cell(0,MaxCellWidth);
if isfield(Data,'GT_Position2')
    [GT2Cell{1:length(Data.GT_Position2.Time),1}]    = deal('point2');
    GT2Cell(:,2)                            = num2cell(Data.GT_Position2.Time);
    GT2Cell(:,3)                            = num2cell(Data.GT_Position2.X);
    GT2Cell(:,4)                            = num2cell(Data.GT_Position2.Y);
    GT2Cell(:,5:8)                          = num2cell(zeros(numel(Data.GT_Position2.Time),4));
end

GTAngleCell = cell(0,MaxCellWidth);
if isfield(Data,'GT_Angle')
    [GTAngleCell{1:length(Data.GT_Angle.Time),1}]    = deal('angle');
    GTAngleCell(:,2)                            = num2cell(Data.GT_Angle.Time);
    GTAngleCell(:,3)                            = num2cell(Data.GT_Angle.Mean);
    GTAngleCell(:,4)                            = num2cell(zeros(numel(Data.GT_Angle.Time), 1));
end

GTPose2Cell = cell(0,MaxCellWidth);
if isfield(Data,'GT_Pose2')
    [GTPose2Cell{1:length(Data.GT_Position2.Time),1}]    = deal('pose2');
    GTPose2Cell(:,2)                            = num2cell(Data.GT_Pose2.Time);
    GTPose2Cell(:,3)                            = num2cell(Data.GT_Pose2.X);
    GTPose2Cell(:,4)                            = num2cell(Data.GT_Pose2.Y);
    GTPose2Cell(:,5)                            = num2cell(Data.GT_Pose2.Yaw);
    GTPose2Cell(:,6:14)                         = num2cell(zeros(numel(Data.GT_Pose2.Time),9));
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
MeasurementCell = [ Pos3Cell;Pos2Cell; PosID2Cell; PosConfID2Cell; BoundingBox3Cell; QuatCell; Speed3Cell;...
                    PoseBetween2Cell;PoseBetween3Cell;...
                    IMUCell;...
                    PressureCell;...
                    Odom2Cell; Odom2DiffCell; Odom3Cell;...
                    Odom3RadarCell; Odom3LaserCell; Odom3VIOCell;...
                    Range2Cell; Range3Cell; RangeLM2Cell; RangeLM3Cell;...
                    Pseudorange2Cell; Pseudorange3Cell;...
                    LoopCell; BearingRangeID2Cell];
                
GTCell = [GT2Cell; GTAngleCell; GTPose2Cell; GT3Cell];

end

