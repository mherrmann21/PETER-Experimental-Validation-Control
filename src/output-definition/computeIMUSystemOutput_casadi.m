function [y_IMU_gyr, y_IMU_acc] = computeIMUSystemOutput_casadi(MBSys, IMUDef,...
        q_k0, q_k, q_k1, g_k, g_k0, g_k1, h)
    %% Compute the outputs of a system in one time step
    arguments
        MBSys       (1,1) MBSystemSym

        % Definitions needed to compute the IMU outputs
        IMUDef      (1,1) MBSysIMUOutputDefinition

        % Variables at time steps k0, k, and k1
        q_k0        (:,1)
        q_k         (:,1)
        q_k1        (:,1)
        g_k         (:,1) SE3
        g_k0        (:,1) SE3
        g_k1        (:,1) SE3

        % Time step
        h           (1,1)
    end

    %% Compute IMU outputs
    f = getSE3Functions(q_k);

    nIMUs = length(IMUDef.s);

    omega_IMU_k = cell(nIMUs, 1);
    x_ddot_IMU  = cell(nIMUs, 1);

    %%% TODO generalize to include link index

    for iIMU = 1:nIMUs
        g_IMU_rel = IMUDef.g_rel(iIMU);

        % Positions of the beam frames (including first fixed node)
        sFrames = [0; cumsum(MBSys.frameData.l)];

        % Get frame before IMU
        iFrIMU = find(IMUDef.s(iIMU) > sFrames, 1, "last");

        % Length difference
        ds = IMUDef.s(iIMU) - sFrames(iFrIMU);

        % Get discrete deformations
        qIndices = double(MBSys.frameData.qIndices(1,iFrIMU-1):MBSys.frameData.qIndices(2,iFrIMU-1));

        Ba = MBSys.frameData.Ba(iFrIMU-1);
        om_k  = (Ba(1:3,:) * q_k (qIndices) + MBSys.frameData.xiC(1:3,iFrIMU-1));
        om_k0 = (Ba(1:3,:) * q_k0(qIndices) + MBSys.frameData.xiC(1:3,iFrIMU-1));
        om_k1 = (Ba(1:3,:) * q_k1(qIndices) + MBSys.frameData.xiC(1:3,iFrIMU-1));
        v_k   = (Ba(4:6,:) * q_k (qIndices) + MBSys.frameData.xiC(4:6,iFrIMU-1));
        v_k0  = (Ba(4:6,:) * q_k0(qIndices) + MBSys.frameData.xiC(4:6,iFrIMU-1));
        v_k1  = (Ba(4:6,:) * q_k1(qIndices) + MBSys.frameData.xiC(4:6,iFrIMU-1));

        % Transformation for the interpolation along s
        g_interp_k  = SE3;
        g_interp_k0 = SE3;
        g_interp_k1 = SE3;

        [g_interp_k.R , g_interp_k.x ] = f.caySE3(om_k *ds, v_k *ds);
        [g_interp_k0.R, g_interp_k0.x] = f.caySE3(om_k0*ds, v_k0*ds);
        [g_interp_k1.R, g_interp_k1.x] = f.caySE3(om_k1*ds, v_k1*ds);

        % IMU configuration
        g_IMU_k  = g_k (iFrIMU-1) * g_interp_k  * g_IMU_rel;
        g_IMU_k0 = g_k0(iFrIMU-1) * g_interp_k0 * g_IMU_rel;
        g_IMU_k1 = g_k1(iFrIMU-1) * g_interp_k1 * g_IMU_rel;

        % IMU velocities
        g_eta_k  = g_IMU_k \ g_IMU_k1;
        g_eta_k0 = g_IMU_k0 \ g_IMU_k;

        [om_k , v_k ] = f.cayInvSE3(g_eta_k.R,  g_eta_k.x);
        [om_k0, v_k0] = f.cayInvSE3(g_eta_k0.R, g_eta_k0.x);

        eta_IMU_k  = vertcat(om_k,  v_k)/h;
        eta_IMU_k0 = vertcat(om_k0, v_k0)/h;

        x_dot_k  = g_IMU_k.R  * eta_IMU_k(4:6);
        x_dot_k0 = g_IMU_k0.R * eta_IMU_k0(4:6);

        x_ddot_s = (x_dot_k - x_dot_k0)/h;
        x_ddot_b = g_IMU_k.R.' * x_ddot_s;

        x_ddot_IMU{iIMU} = x_ddot_b + g_IMU_k.R.'*[0;0; 9.81];

        omega_IMU_k{iIMU} = eta_IMU_k(1:3);
    end

    y_IMU_gyr = omega_IMU_k;
    y_IMU_acc = x_ddot_IMU;

end

