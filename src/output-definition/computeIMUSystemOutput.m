function [y_IMU_gyr, y_IMU_acc] = computeIMUSystemOutput(MBSys, IMUDef, ...
        q_k, q_k0, q_k1, g_k, g_k0, g_k1, h)
    %% Compute the IMU outputs of a system in one time step
    arguments
        MBSys       (1,1) MBSystem

        % Definitions needed to compute the IMU outputs
        IMUDef      (1,1) MBSysIMUOutputDefinition

        % Variables at time steps k and k0
        q_k         (:,1)
        q_k0        (:,1)
        q_k1        (:,1)
        g_k         (4,4,:)
        g_k0        (4,4,:)
        g_k1        (4,4,:)

        % Time step size
        h           (1,1)
    end

    %% Compute IMU outputs

    gGrav = 9.807232; % Munich

    nIMUs = length(IMUDef.s);
    eta_IMU_k = zeros(6, nIMUs);
    x_ddot_IMU = zeros(3, nIMUs);

    %%% TODO generalize to include link index

    for iIMU = 1:nIMUs
        g_IMU_rel = IMUDef.g_rel(:,:,iIMU);

        %%% TODO Get frames for IMU link here

        % Positions of the beam frames (including first fixed node)
        sFrames = [0; cumsum(MBSys.frameData.l)];

        % Get frame before IMU
        iFrIMU = find(IMUDef.s(iIMU) > sFrames, 1, "last");

        % Length difference
        ds = IMUDef.s(iIMU) - sFrames(iFrIMU);

        % Get discrete deformations
        xi_k  = MBSys.getLinkDeformations(q_k, 1);
        xi_k0 = MBSys.getLinkDeformations(q_k0, 1);
        xi_k1 = MBSys.getLinkDeformations(q_k1, 1);

        g_IMU_k  = g_k (:,:,iFrIMU-1) * caySE3(xi_k (:,iFrIMU-1)*ds)*g_IMU_rel;
        g_IMU_k0 = g_k0(:,:,iFrIMU-1) * caySE3(xi_k0(:,iFrIMU-1)*ds)*g_IMU_rel;
        g_IMU_k1 = g_k1(:,:,iFrIMU-1) * caySE3(xi_k1(:,iFrIMU-1)*ds)*g_IMU_rel;

        % IMU velocities
        eta_IMU_k(:,iIMU) = cayInvSE3(g_IMU_k\g_IMU_k1)/h;
        eta_IMU_k0        = cayInvSE3(g_IMU_k0\g_IMU_k)/h;

        x_dot_k  = g_IMU_k(1:3,1:3)  * eta_IMU_k(4:6, iIMU);
        x_dot_k0 = g_IMU_k0(1:3,1:3) * eta_IMU_k0(4:6);

        x_ddot_s = (x_dot_k - x_dot_k0)/h;
        x_ddot_b = g_IMU_k(1:3,1:3).' * x_ddot_s;

        x_ddot_IMU(:,iIMU) = x_ddot_b + g_IMU_k(1:3,1:3).'*[0;0; gGrav];
    end

    y_IMU_gyr = eta_IMU_k(1:3,:);
    y_IMU_acc = x_ddot_IMU;
end