function [IMUDef, cableDef, IMUParams] = definePETEROutputs(link)
    %% Define System outputs for PETER measurements
    arguments
        link (1,1) MBLinkDefinition
    end

    %% Define cable outputs

    % Disk arc length positions
    sDisks = [0, linspace(0.042,0.639,13), 0.685];

    cableDef = MBSysTendonLengthOutputDefinition(link.cableConfig, sDisks);


    %% Define IMU outputs
    IMUDef = MBSysIMUOutputDefinition;
    IMUDef.s = [0.537, 0.634];
    nIMUs = 2;

    % Transformation CS frame -> CoM IMU
    % Specify via polar coordinates
    phi_IMU_vec = [1, 3] + 180; % Measured from x-axis
    r_IMU = 23.6e-3;

    % Constant rotation matrix that transforms to IMU frame convention
    Rconst = [
        0  -1 0
        -1 0  0
        0  0 -1
        ];

    % SE3 transformation from cross-section frame to IMU frame
    IMUDef.g_rel = zeros(4,4, nIMUs);

    for iIMU = 1:nIMUs
        % Transformation to IMU position and rotation about CS z-axis
        phi_IMU = phi_IMU_vec(iIMU);
        R_IMU_rel = [
            cosd(phi_IMU), -sind(phi_IMU), 0;
            sind(phi_IMU), +cosd(phi_IMU), 0;
            0            , 0             , 1];

        % Configuration without z-axis rotation
        g_IMU_0 = SE3Matrix(Rconst, [r_IMU; 0; 0]);

        % Additional relative z-axis rotation
        g_rot_rel = SE3Matrix(R_IMU_rel, zeros(3,1));

        % Overall configuration
        IMUDef.g_rel(:,:,iIMU) = g_rot_rel * g_IMU_0;

        %x_IMU_rel = R_IMU_rel * [r_IMU; 0; 0];

        % Additional transformation to the IMU measurement frame
        %R_IMU_rel_fr = R_IMU_rel * Rconst;

    end

    IMUParams.Rconst = Rconst;
    IMUParams.phi = phi_IMU_vec;
    IMUParams.r = r_IMU;

end