function [ResultStruct] = parseFromCell(ResultCell)
%PARSEFROMCELL parse cell (from csv) to data structure
ResultStruct = struct;

for n = 1:size(ResultCell,1)
    switch ResultCell{n,1}

        case 'point1'
            if ~isfield(ResultStruct,'Position1')
                ResultStruct.Position1 = [];
                ResultStruct.Position1.Time = [];
                ResultStruct.Position1.X = [];
                ResultStruct.Position1.Cov = [];
            end
            ResultStruct.Position1.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.Position1.X(end+1,1) = cell2mat(ResultCell(n,3));
            ResultStruct.Position1.Cov(end+1,:,:) = reshape(cell2mat(ResultCell(n,4)),[],1,1);

        case 'point2'
            if ~isfield(ResultStruct,'Position2')
                ResultStruct.Position2 = [];
                ResultStruct.Position2.Time = [];
                ResultStruct.Position2.X = [];
                ResultStruct.Position2.Y = [];
                ResultStruct.Position2.Cov = [];
            end
            ResultStruct.Position2.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.Position2.X(end+1,1) = cell2mat(ResultCell(n,3));
            ResultStruct.Position2.Y(end+1,1) = cell2mat(ResultCell(n,4));
            ResultStruct.Position2.Cov(end+1,:,:) = reshape(cell2mat(ResultCell(n,5:8)),[],2,2);

        case 'pose2'
            if ~isfield(ResultStruct,'Pose2')
                ResultStruct.Pose2 = [];
                ResultStruct.Pose2.Time = [];
                ResultStruct.Pose2.X = [];
                ResultStruct.Pose2.Y = [];
                ResultStruct.Pose2.Yaw = [];
                ResultStruct.Pose2.Cov = [];
            end
            ResultStruct.Pose2.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.Pose2.X(end+1,1) = cell2mat(ResultCell(n,3));
            ResultStruct.Pose2.Y(end+1,1) = cell2mat(ResultCell(n,4));
            ResultStruct.Pose2.Yaw(end+1,1) = cell2mat(ResultCell(n,5));
            ResultStruct.Pose2.Cov(end+1,:,:) = reshape(cell2mat(ResultCell(n,6:14)),[],3,3);

        case 'point_id2'
            if ~isfield(ResultStruct,'PositionID2')
                ResultStruct.PositionID2 = [];
                ResultStruct.PositionID2.Time = [];
                ResultStruct.PositionID2.X = [];
                ResultStruct.PositionID2.Y = [];
                ResultStruct.PositionID2.ID = [];
                ResultStruct.PositionID2.Cov = [];
            end
            ResultStruct.PositionID2.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.PositionID2.X(end+1,1) = cell2mat(ResultCell(n,3));
            ResultStruct.PositionID2.Y(end+1,1) = cell2mat(ResultCell(n,4));
            ResultStruct.PositionID2.ID(end+1,1) = cell2mat(ResultCell(n,5));
            ResultStruct.PositionID2.Cov(end+1,:,:) = reshape(cell2mat(ResultCell(n,6:9)),[],2,2);

        case 'bounding_box_3'
            if ~isfield(ResultStruct,'BoundingBox3')
                ResultStruct.BoundingBox3 = [];
                ResultStruct.BoundingBox3.Time = [];
                ResultStruct.BoundingBox3.X = [];
                ResultStruct.BoundingBox3.Y = [];
                ResultStruct.BoundingBox3.Z = [];
                ResultStruct.BoundingBox3.ID = [];
                ResultStruct.BoundingBox3.Idx = [];
                ResultStruct.BoundingBox3.Conf = [];
                ResultStruct.BoundingBox3.Cov = [];
                ResultStruct.BoundingBox3.WLH = [];
                ResultStruct.BoundingBox3.R = [];
                ResultStruct.BoundingBox3.R_Quat = [];
                ResultStruct.BoundingBox3.ClassNum = [];
                ResultStruct.BoundingBox3.KeyNum = [];
                ResultStruct.BoundingBox3.Velocity = [];
            end
            ResultStruct.BoundingBox3.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.BoundingBox3.X(end+1,1) = cell2mat(ResultCell(n,3));
            ResultStruct.BoundingBox3.Y(end+1,1) = cell2mat(ResultCell(n,4));
            ResultStruct.BoundingBox3.Z(end+1,1) = cell2mat(ResultCell(n,5));
            ResultStruct.BoundingBox3.Velocity(end+1,:)   = cell2mat(ResultCell(n,6:8));
            ResultStruct.BoundingBox3.ID(end+1,1) = cell2mat(ResultCell(n,9));
            ResultStruct.BoundingBox3.Idx(end+1,1) = cell2mat(ResultCell(n,10));
            ResultStruct.BoundingBox3.Conf(end+1,1) = cell2mat(ResultCell(n,11));
            ResultStruct.BoundingBox3.Cov(end+1,:,:) = reshape(cell2mat(ResultCell(n,12:20)),[],3,3);
            ResultStruct.BoundingBox3.WLH(end+1,:)   = cell2mat(ResultCell(n,21:23));
            ResultStruct.BoundingBox3.R(end+1,:)   = cell2mat(ResultCell(n,24));
            ResultStruct.BoundingBox3.R_Quat(end+1,:)   = cell2mat(ResultCell(n,25:28));
            ResultStruct.BoundingBox3.ClassNum(end+1,:)   = cell2mat(ResultCell(n,29));
            ResultStruct.BoundingBox3.KeyNum(end+1,:)   = cell2mat(ResultCell(n,30));
            
        case 'point3'
            if ~isfield(ResultStruct,'Position3')
                ResultStruct.Position3 = [];
                ResultStruct.Position3.Time = [];
                ResultStruct.Position3.X = [];
                ResultStruct.Position3.Y = [];
                ResultStruct.Position3.Z = [];
                ResultStruct.Position3.Cov = [];
            end
            ResultStruct.Position3.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.Position3.X(end+1,1) = cell2mat(ResultCell(n,3));
            ResultStruct.Position3.Y(end+1,1) = cell2mat(ResultCell(n,4));
            ResultStruct.Position3.Z(end+1,1) = cell2mat(ResultCell(n,5));
            ResultStruct.Position3.Cov(end+1,:,:) = reshape(cell2mat(ResultCell(n,6:14)),[],3,3);

        case 'quaternion'
            if ~isfield(ResultStruct,'Quaternion')
                ResultStruct.Quaternion = [];
                ResultStruct.Quaternion.Time = [];
                ResultStruct.Quaternion.X = [];
                ResultStruct.Quaternion.Y = [];
                ResultStruct.Quaternion.Z = [];
                ResultStruct.Quaternion.W = [];
                ResultStruct.Quaternion.Cov = [];
            end
            ResultStruct.Quaternion.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.Quaternion.X(end+1,1) = cell2mat(ResultCell(n,3));
            ResultStruct.Quaternion.Y(end+1,1) = cell2mat(ResultCell(n,4));
            ResultStruct.Quaternion.Z(end+1,1) = cell2mat(ResultCell(n,5));
            ResultStruct.Quaternion.W(end+1,1) = cell2mat(ResultCell(n,6));
            ResultStruct.Quaternion.Cov(end+1,:,:) = reshape(cell2mat(ResultCell(n,7:22)),[],4,4);

        case 'angle'
            if ~isfield(ResultStruct,'Angle')
                ResultStruct.Angle = [];
                ResultStruct.Angle.Time = [];
                ResultStruct.Angle.Mean = [];
                ResultStruct.Angle.Cov = [];
            end
            ResultStruct.Angle.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.Angle.Mean(end+1,1) = cell2mat(ResultCell(n,3));
            ResultStruct.Angle.Cov(end+1,1) = cell2mat(ResultCell(n,4));

        case 'unit_circle'
            if ~isfield(ResultStruct,'UnitCircle')
                ResultStruct.UnitCircle = [];
                ResultStruct.UnitCircle.Time = [];
                ResultStruct.UnitCircle.Real = [];
                ResultStruct.UnitCircle.Complex = [];
                ResultStruct.UnitCircle.Cov = [];
            end
            ResultStruct.UnitCircle.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.UnitCircle.Real(end+1,1) = cell2mat(ResultCell(n,3));
            ResultStruct.UnitCircle.Complex(end+1,1) = cell2mat(ResultCell(n,4));
            ResultStruct.UnitCircle.Cov(end+1,:,:) = reshape(cell2mat(ResultCell(n,5:8)),[],2,2);

        case 'imu_bias'
            if ~isfield(ResultStruct,'IMU')
                ResultStruct.IMU = [];
                ResultStruct.IMU.Time = [];
                ResultStruct.IMU.Speed = [];
                ResultStruct.IMU.AccBias = [];
                ResultStruct.IMU.TRBias = [];
            end
            ResultStruct.IMU.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.IMU.Speed(end+1,:) = cell2mat(ResultCell(n,3:5));
            ResultStruct.IMU.AccBias(end+1,:) = cell2mat(ResultCell(n,6:8));
            ResultStruct.IMU.TRBias(end+1,:) = cell2mat(ResultCell(n,9:11));

        case 'error1'
            if ~isfield(ResultStruct,'Error1')
                ResultStruct.Error1 = [];
                ResultStruct.Error1.Time = [];
                ResultStruct.Error1.Mean = [];
            end
            ResultStruct.Error1.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.Error1.Mean(end+1,1) = cell2mat(ResultCell(n,3));

        case 'error2'
            if ~isfield(ResultStruct,'Error2')
                ResultStruct.Error2 = [];
                ResultStruct.Error2.Time = [];
                ResultStruct.Error2.Mean = [];
            end
            ResultStruct.Error2.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.Error2.Mean(end+1,1:2) = cell2mat(ResultCell(n,3:4));

        case 'error3'
            if ~isfield(ResultStruct,'Error3')
                ResultStruct.Error3 = [];
                ResultStruct.Error3.Time = [];
                ResultStruct.Error3.Mean = [];
            end
            ResultStruct.Error3.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.Error3.Mean(end+1,1:3) = cell2mat(ResultCell(n,3:5));


        case 'error6'
            if ~isfield(ResultStruct,'Error6')
                ResultStruct.Error6 = [];
                ResultStruct.Error6.Time = [];
                ResultStruct.Error6.Mean = [];
            end
            ResultStruct.Error6.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.Error6.Mean(end+1,1:6) = cell2mat(ResultCell(n,3:8));

        case 'cost_gradient1'
            if ~isfield(ResultStruct,'Cost1')
                ResultStruct.Cost1 = [];
                ResultStruct.Cost1.Time = [];
                ResultStruct.Cost1.Position = [];
                ResultStruct.Cost1.Cost = [];
                ResultStruct.Cost1.Gradient = [];
                ResultStruct.Cost1.Hessian = [];
            end
            ResultStruct.Cost1.Time(end+1,1)     = cell2mat(ResultCell(n,2));
            ResultStruct.Cost1.Position(end+1,1) = cell2mat(ResultCell(n,3));
            ResultStruct.Cost1.Cost(end+1,1)     = cell2mat(ResultCell(n,4));
            ResultStruct.Cost1.Gradient(end+1,1) = cell2mat(ResultCell(n,5));
            ResultStruct.Cost1.Hessian(end+1,1) = cell2mat(ResultCell(n,6));

        case 'cost_gradient2'
            if ~isfield(ResultStruct,'Cost2')
                ResultStruct.Cost2 = [];
                ResultStruct.Cost2.Time = [];
                ResultStruct.Cost2.Position = [];
                ResultStruct.Cost2.Cost = [];
                ResultStruct.Cost2.Gradient = [];
                ResultStruct.Cost2.Hessian = [];
            end
            ResultStruct.Cost2.Time(end+1,1)     = cell2mat(ResultCell(n,2));
            ResultStruct.Cost2.Position(end+1,:) = cell2mat(ResultCell(n,3:4));
            ResultStruct.Cost2.Cost(end+1,1)     = cell2mat(ResultCell(n,5));
            ResultStruct.Cost2.Gradient(end+1,:) = cell2mat(ResultCell(n,6:7));
            ResultStruct.Cost2.Hessian(end+1,:) = cell2mat(ResultCell(n,8:11));

        case 'cost_gradient3'
            if ~isfield(ResultStruct,'Cost3')
                ResultStruct.Cost3 = [];
                ResultStruct.Cost3.Time = [];
                ResultStruct.Cost3.Position = [];
                ResultStruct.Cost3.Cost = [];
                ResultStruct.Cost3.Gradient = [];
                ResultStruct.Cost3.Hessian = [];
            end
            ResultStruct.Cost3.Time(end+1,1)     = cell2mat(ResultCell(n,2));
            ResultStruct.Cost3.Position(end+1,:) = cell2mat(ResultCell(n,3:5));
            ResultStruct.Cost3.Cost(end+1,1)     = cell2mat(ResultCell(n,6));
            ResultStruct.Cost3.Gradient(end+1,:) = cell2mat(ResultCell(n,7:9));
            ResultStruct.Cost3.Hessian(end+1,:) = cell2mat(ResultCell(n,10:13));

        case 'solver_summary'
            if ~isfield(ResultStruct,'SolverSummary')
                ResultStruct.SolverSummary = [];
                ResultStruct.SolverSummary.Time = [];
                ResultStruct.SolverSummary.DurationTotal = [];
                ResultStruct.SolverSummary.DurationSolver = [];
                ResultStruct.SolverSummary.DurationCovariance = [];
                ResultStruct.SolverSummary.DurationMarginal = [];
                ResultStruct.SolverSummary.DurationAdaptive = [];
                ResultStruct.SolverSummary.IterationSolver = [];
                ResultStruct.SolverSummary.IterationAdaptive = [];
            end
            ResultStruct.SolverSummary.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.SolverSummary.DurationTotal(end+1,1) = cell2mat(ResultCell(n,3));
            ResultStruct.SolverSummary.DurationSolver(end+1,1) = cell2mat(ResultCell(n,4));
            ResultStruct.SolverSummary.DurationCovariance(end+1,1) = cell2mat(ResultCell(n,5));
            ResultStruct.SolverSummary.DurationMarginal(end+1,1) = cell2mat(ResultCell(n,6));
            ResultStruct.SolverSummary.DurationAdaptive(end+1,1) = cell2mat(ResultCell(n,7));
            ResultStruct.SolverSummary.IterationSolver(end+1,1) = cell2mat(ResultCell(n,8));
            ResultStruct.SolverSummary.IterationAdaptive(end+1,1) = cell2mat(ResultCell(n,9));

        case 'cov2'
            if ~isfield(ResultStruct,'Covariance2')
                ResultStruct.Covariance2 = [];
                ResultStruct.Covariance2.Time = [];
                ResultStruct.Covariance2.Mean = [];
                ResultStruct.Covariance2.Cov = [];
            end
            ResultStruct.Covariance2.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.Covariance2.Mean(end+1,1:2) = cell2mat(ResultCell(n,3:4));
            ResultStruct.Covariance2.Cov(end+1,:,:) = reshape(cell2mat(ResultCell(n,5:8)),[],2,2);

        case 'gmm1'
            if ~isfield(ResultStruct,'GMM')
                ResultStruct.GMM = [];
                ResultStruct.GMM.Time = [];
                ResultStruct.GMM.Mean = [];
                ResultStruct.GMM.Cov = [];
                ResultStruct.GMM.Weight = [];
            end
            NumComp = sum(~ismissing([ResultCell{n,3:end}]))/3;
            ResultStruct.GMM.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.GMM.Mean(end+1,1:NumComp) = cell2mat(ResultCell(n,2 + (1:NumComp)));
            ResultStruct.GMM.Cov(end+1,1:NumComp) = cell2mat(ResultCell(n,2+NumComp + (1:NumComp)));
            ResultStruct.GMM.Weight(end+1,1:NumComp) = cell2mat(ResultCell(n,2+NumComp*2 + (1:NumComp)));

        case 'mmd'
            if ~isfield(ResultStruct,'MMD')
                ResultStruct.MMD = [];
                ResultStruct.MMD.Time = [];
                ResultStruct.MMD.Mean = [];
            end
            ResultStruct.MMD.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.MMD.Mean(end+1,1) = cell2mat(ResultCell(n,3));

        case 'sinkhorn'
            if ~isfield(ResultStruct,'Sinkhorn')
                ResultStruct.Sinkhorn = [];
                ResultStruct.Sinkhorn.Time = [];
                ResultStruct.Sinkhorn.Loss = [];
            end
            ResultStruct.Sinkhorn.Time(end+1,1) = cell2mat(ResultCell(n,2));
            ResultStruct.Sinkhorn.Loss(end+1,1) = cell2mat(ResultCell(n,3));
        otherwise
            error(['Wrong StateName: ' ResultCell{n,1}]);
    end
end
end

