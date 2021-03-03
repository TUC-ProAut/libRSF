function [qNew] = resampleQuaternionSlerp(tOld, qOld, tNew)
%RESAMPLEQUATERNIONSLERP Interpolates using spherical linear interpolation
% INPUT
%   tOld - nx1 - timestamps for input quaternions
%   qOld - nx1 quaternion - input quaternions
%   tNew - mx1 - timestamps for resampled quaternions
%
% OUTPUT
%   qNew - mx1 quaternion - resampled quaternions

q1Idx = zeros(length(tNew),1);
q2Idx = zeros(length(tNew),1);

q2IdxTmp = 1;
oldEndReached = 0; % flag that is set if current tNew goes past the last tOld

for kNew = 1:length(tNew)
    % increment "second" quaternion idx if we are past the time of the 
    % current "second" quaternion
    % loop for jumping multiple quaternion idx when downsampling or to
    % bridge gaps
    while tNew(kNew) > tOld(q2IdxTmp)
        if q2IdxTmp < length(tOld) % make sure we dont get out of bounds
            q2IdxTmp = q2IdxTmp + 1;
        else
        % (manage nearest neighbor extrapolation after last tOld)
            oldEndReached = 1;
            break;
        end
    end

    % write second quaternion idx    
    q2Idx(kNew) = q2IdxTmp;

    % write first quaternion idx
    % (manage nearest neighbor extrapolation before first tOld)
    if q2IdxTmp > 1  
         % (manage nearest neighbor extrapolation after last tOld border)
        if ~oldEndReached
            q1Idx(kNew) = q2IdxTmp-1;
        else
            q1Idx(kNew) = q2IdxTmp;
        end
    else
        q1Idx(kNew) = 1;
    end
end

qNew = quaternion(zeros(length(tNew),4));
parfor kNew = 1:length(tNew)
    % "extrapolate" using nearest neighbor
    if q1Idx(kNew) == q2Idx(kNew)
        qNew(kNew) = qOld(q1Idx(kNew));
    % interpolate using SLERP
    else
        T = (tNew(kNew)-tOld(q1Idx(kNew)))./(tOld(q2Idx(kNew))-tOld(q1Idx(kNew)));
        qNew(kNew) = slerp(qOld(q1Idx(kNew)), qOld(q2Idx(kNew)), T);
    end   
end
end
