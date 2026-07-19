function pars = getNominalPeterBaseParams()
    %% Compute PETER Nominal "Base" Parameters

    pars = struct;

    %% Inertia calculation

    % BOQA fastener
    m = 1.62e-3;
    l = 4e-3;
    r = 3e-3; % rough approximation

    [BQ.I_z, BQ.I_xy] = getCylInertia(m,r,l);
    BQ.m = m;

    % Standard disks
    m = 4e-3;
    r = 23e-3;
    l = 2.5e-3;
    [I_z, I_xy] = getCylInertia(m,r,l);

    pars.StandardDisk.m    = m + BQ.m;
    pars.StandardDisk.I_z  = I_z  + BQ.I_z;
    pars.StandardDisk.I_xy = I_xy + BQ.I_xy;
    pars.StandardDisk.MGen = blkdiag(diag([ ...
        pars.StandardDisk.I_xy, pars.StandardDisk.I_xy, pars.StandardDisk.I_z
        ]), eye(3) * pars.StandardDisk.m);

    % Tip disk
    m = 10e-3;
    r = 23e-3;
    l = 8e-3;
    [I_z, I_xy] = getCylInertia(m,r,l);

    pars.TipDisk.m    = m + BQ.m;
    pars.TipDisk.I_z  = I_z  + BQ.I_z;
    pars.TipDisk.I_xy = I_xy + BQ.I_xy;
    pars.TipDisk.MGen = blkdiag(diag([ ...
        pars.TipDisk.I_xy, pars.TipDisk.I_xy, pars.TipDisk.I_z
        ]), eye(3) * pars.TipDisk.m);



    %% IMUs / IMU disks

    % IMU weight alone
    % Includes board and the additional spacer disk weight w.r.t. a
    % standard spacer disk
    pars.IMUs.m = ones(2,1) * (1.5e-3 + 0.65e-3);

    % Full spacer disk
    pars.IMUDisk.m = ones(2,1) * (4.66e-3 + 1.5e-3);
    pars.IMUDisk.MGen = ones(2,1) * (4.66e-3 + 1.5e-3);

    % IMU position
    sIMUs = [0.636, 0.539];

    % IMU disks
    % Rotate CoM vector using polar coordinates
    phi_IMU_vec = [5, 5] + 180; % Measured from x-axis
    pars.IMUs.xCoM = zeros(3, length(sIMUs));

    for iIMU = 1:length(sIMUs)
        phi_IMU = phi_IMU_vec(iIMU);
        R_IMU_rel = [
            cosd(phi_IMU), -sind(phi_IMU), 0;
            sind(phi_IMU), +cosd(phi_IMU), 0;
            0            , 0             , 1];

        pars.IMUs.xCoM(:,iIMU) = R_IMU_rel * [20e-3; 0; 0];
        pars.IMUDisk.xCoM(:,iIMU) = R_IMU_rel * [4e-3; 0; 0];
    end
    pars.IMUs.sAtt = sIMUs.';
    pars.IMUDisk.sAtt = sIMUs.';


    %% IMU tendon weight calculation

    % Tendon weight per length (specific mass)
    m_tendon_sp = 8.09 / 44;

    % Lengths of the tendon segments (top to bottom)
    tendonLengths = [
        160
        190
        170
        180
        ]*1e-3;

    % Spacer disk segment length
    lSD = 0.7/14;

    % Center of mass points of the tendon segments
    sTendonCoMs = [12, 9.5, 6.5, 3.5] * lSD;

    % CoM points of the tendon segments, computed via angle and radius
    alphaTendon = 107 + [0, 0, 0, 0];

    % Masses and CoMs of the tendon *segments*
    m_tendonSeg = tendonLengths * m_tendon_sp;
    xCoM_tendonSegs = zeros(3, length(tendonLengths));
    for iT = 1:length(tendonLengths)
        alpha = alphaTendon(iT);
        R_t_rel = [
            cosd(alpha), -sind(alpha), 0;
            sind(alpha), +cosd(alpha), 0;
            0            , 0         , 1];

        xCoM_tendonSegs(:,iT) = R_t_rel * ...
            [30e-3; 0; 0];
    end

    % Points where the tendons are attached
    sTendonAtt = [13,11,8,5,2] * lSD;

    % Compute contributions to the spacer disks
    pars.IMUTendons.xCoMs = zeros(3, 0);
    pars.IMUTendons.m    = zeros(0,1);
    pars.IMUTendons.sAtt = zeros(0,1);

    for iT = 1:length(tendonLengths)
        pars.IMUTendons.m(end+1,1) = m_tendonSeg(iT)/2;
        pars.IMUTendons.m(end+1,1) = m_tendonSeg(iT)/2;

        pars.IMUTendons.sAtt(end+1,1) = sTendonAtt(iT);
        pars.IMUTendons.sAtt(end+1,1) = sTendonAtt(iT+1);

        pars.IMUTendons.xCoMs(:,end+1) = [
            xCoM_tendonSegs(1:2,iT);
            sTendonCoMs(iT) - sTendonAtt(iT)
            ];
        pars.IMUTendons.xCoMs(:,end+1) = [
            xCoM_tendonSegs(1:2,iT);
            sTendonCoMs(iT) - sTendonAtt(iT+1)
            ];
    end
   
    %% Local functions

    function [I_z, I_xy] = getCylInertia(m,r,l)
        I_z = 1/2*m*r^2;
        I_xy = 1/4 *m*r^2 + 1/12*m*l^2;
    end
end
