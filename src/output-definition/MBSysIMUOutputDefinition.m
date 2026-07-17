classdef MBSysIMUOutputDefinition
    %% System output definition for IMUs mounted on a link
    properties
        % Index of the MBSystem link, to which the IMU is attached
        % dimensions (nIMUs, 1)
        linkIndex   (:,1) double {mustBePositive} = 1;

        % Arc length position along the center line (if the link is
        % flexible)
        % dimensions (nIMUs, 1)
        s           (1,:) double

        % Relative transformation to the IMU
        %  * For flexible links: From the center line frame at s to the IMU
        %    (if there is no beam node directly at s, the centerline
        %    position at s is computed via SE3 interpolation)
        %  * For rigid links, from the link's reference frame to the IMU
        % dimensions (4,4,nIMUs) or SE3 array
        g_rel       %(4,4,:) double

        % Calibration parameters (only for parameter identification)
        rCalib
        xCalib
    end
end