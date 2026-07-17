function [link, IMUDef] = systemDef_PETER_nominal_reduced(opts)
    %% Define MBS System: PETER / continuum manipulator
    % With nominal parameters and reduced number of segments (i.e.,
    % segments do not correspond to spacer disk segments)
    arguments
        opts.d      (6,1) double = ones(6,1)*9.0e-3;
        opts.nSeg   (1,1) uint8  = 7;
        opts.usedTendons (:,1) double = 1:4;
    end

    %% Model calibration/tuning factors
    fE = 1.14;                                          % Young's modulus
    fExtMasses = 0.32;                                  % External masses
    fTipMass = 0.3;                                     % Tip mass
    fSD = 1.0;                                          % Spacer disks
    fExtCoMs = 0.9;                                     % CoM positions of external masses
    phiT = [-3.25, -3, -3.75];                          % Polar offset angle of straight tendons
    alpha_cables = 80;                                  % Polar angle specifying the IMU caple position
    xiRef = repmat([0;0.06;0;0;0;1], [1,opts.nSeg]);    % Reference curvature


    %% Basic Link Configuration
    link = MBLinkDefinitionFlexible;

    link.parentLink   = 0;
    link.isCantilever = true;
    link.isActuated   = false;
    link.nSeg         = opts.nSeg;
    link.L            = 0.685;
    link.g_J_B        = eye(4);
    link.Ba = [ eye(3); zeros(3)];
    link.Bc = [ zeros(3); eye(3)];
    link.xiRef = xiRef;
    link.beamPars   = beamParams_spring_steel_round("radius", 1e-3, "factorE", fE);
    link.beamPars.d = opts.d;


    %% Define Tendon Path Functions

    % Functions that define the cable path by returning the x,y coordinates
    % of the tendon location in the cross-section plane

    % Straight path
    % The path in the cross-section plane is defined in polar coordinates
    % by distance from backbone d (m) and angle alpha (deg)
    x_m_fun_straight = @(s,d,alpha) [d*cosd(alpha); d*sind(alpha); 0 ];

    % Helical path, taken from [RW11], Table 1
    % Parameters d and p, offset/phase o
    x_m_fun_helical = @(s, d, p, o) [
        d* cos(-2*pi/p*s + o);
        d* sin(-2*pi/p*s + o);
        0
        ];


    %% Set Up Cable Configuration

    % Cell array of function handles; defines the individual cable paths
    x_m_funs = {
        @(s)x_m_fun_straight(s,0.02, 0 + phiT(1))
        @(s)x_m_fun_straight(s,0.02, 120 + phiT(2))
        @(s)x_m_fun_straight(s,0.02, 240 + phiT(3))
        @(s)x_m_fun_helical(s, 0.02, link.L, deg2rad(-60))
        };

    link.cableConfig.x_m_funs = x_m_funs(opts.usedTendons);

    % Compute derivatives of cable path functions
    link.cableConfig = link.cableConfig.getSymbolicPathDerivatives;

    % Lengths at which the cables terminate along the link length
    LTermination = [
        link.L, link.L, link.L, link.L
        ];

    link.cableConfig.LTermination = LTermination(opts.usedTendons);


    %% Add external masses
    % For spacer disks / end effector / payload

    l = link.L/link.nSeg;
    pars = getNominalPeterBaseParams();

    link.g_a = repmat(eye(4), [1,1,link.nSeg+1]);
    link.m_a = zeros(link.nSeg+1, 1);
    link.M_a = zeros(6,6,link.nSeg+1);

    %%% Add tip disk
    pars.TipDisk.MGen = pars.TipDisk.MGen * fTipMass;
    pars.TipDisk.m = pars.TipDisk.m * fTipMass;

    % Transformation to tip disk CoM
    g_TD = SE3Matrix(eye(3), [0; 0; 4e-3]);

    % Complete generalized inertia tensor w.r.t. node frame
    MGen_TD = lAdSE3Inv(g_TD).' * pars.TipDisk.MGen * lAdSE3Inv(g_TD);
    
    link.m_a(end)     = link.m_a(end) + pars.TipDisk.m;
    link.M_a(:,:,end) = link.M_a(:,:,end) + MGen_TD;
    link.g_a(:,:,end) = g_TD;


    %% Add standard spacer disks
    % distribute them to nearest beam nodes

    % Standard spacer disks as discrete masses
    sDisks = linspace(0.043, 0.639, 13);
    sStDisks = sDisks;

    mStDisk = pars.StandardDisk.m *fSD;
    MGenStDisk = pars.StandardDisk.MGen * fSD;

    sFrames = 0:l:link.L;

    DEBUG_PRINT = false;

    % Distribute standard spacer disks
    for iMass = 1:length(sStDisks)
        % Get the distance and indices to the neighboring frames,
        % to which the mass is distributed
        % (the two affected frames correspond to the first two indices in
        % the arrays)
        [frmDistsAbs, frmIdx] = sort(abs(sFrames - sStDisks(iMass)));

        if DEBUG_PRINT
            fprintf("Assigning mass %d at s=%.2f to frames %d (s=%.2f) and %d (s=%.2f)\n", ...
                iMass, sStDisks(iMass), ...
                frmIdx(1), sFrames(frmIdx(1)), ...
                frmIdx(2), sFrames(frmIdx(2)));
            fprintf(" Contribution frame %d: %.3f\n", frmIdx(1), (1-frmDistsAbs(1)/l))
            fprintf(" Contribution frame %d: %.3f\n", frmIdx(2), (1-frmDistsAbs(2)/l))
        end

        % Compute quantities for the two affected frames
        for iFrm = 1:2

            % Compute mass contribution
            m_i    = mStDisk    * (1-frmDistsAbs(iFrm)/l);
            MGen_i = MGenStDisk * (1-frmDistsAbs(iFrm)/l);

            % Assign masses
            link.m_a(frmIdx(iFrm)) = link.m_a(frmIdx(iFrm)) + m_i;
            link.M_a(:,:,frmIdx(iFrm)) = link.M_a(:,:,frmIdx(iFrm)) + MGen_i;
        end

    end

    %% Add evenly distributed cable mass
    sCables = sStDisks(1:end);
    mCables = sum(pars.IMUCables.m);
    mCablesSD = ones(length(sCables),1) * mCables / length(sCables);

    R_c_rel = [
        cosd(alpha_cables), -sind(alpha_cables), 0;
        sind(alpha_cables), +cosd(alpha_cables), 0;
        0            , 0         , 1];

    xCoM_cableSeg = R_c_rel * ...
        [30e-3; 0; 0];

    xCoM_cables = repmat(xCoM_cableSeg, [1,length(sDisks)]);



    %% Add additional masses to frames (as rigid point masses)
    % Masses/inertias are distributed according to their contributions to
    % the neighboring frames

    extMasses = [
        pars.IMUs.m;
        %pars.IMUCables.m
        mCablesSD
        ]*fExtMasses;

    extCoMs = [
        pars.IMUs.xCoM, ...
        xCoM_cables
        %pars.IMUCables.xCoMs
        ]*fExtCoMs;

    ext_s = [
        pars.IMUs.sAtt
        %pars.IMUCables.sAtt
        sCables.'
        ];

    for iMass = 1:length(extMasses)
        % Get the distance and indices to the neighboring frames,
        % to which the mass is distributed
        % (the two affected frames correspond to the first two indices in
        % the arrays)
        [frmDistsAbs, frmIdx] = sort(abs(sFrames - ext_s(iMass)));

        if DEBUG_PRINT
            fprintf("Assigning mass %d at s=%.2f to frames %d (s=%.2f) and %d (s=%.2f)\n", ...
                iMass, ext_s(iMass), ...
                frmIdx(1), sFrames(frmIdx(1)), ...
                frmIdx(2), sFrames(frmIdx(2)));
            fprintf(" Contribution frame %d: %.3f\n", frmIdx(1), (1-frmDistsAbs(1)/l))
            fprintf(" Contribution frame %d: %.3f\n", frmIdx(2), (1-frmDistsAbs(2)/l))
        end

        % Compute quantities for the two affected frames
        for iFrm = 1:2

            % Compute mass contribution
            m_i = extMasses(iMass) * (1-frmDistsAbs(iFrm)/l);

            % Compute CoM vector from frame origin to ext. mass
            xCoM_i = [extCoMs(1:2, iMass); 0];

            % Compute new overall CoM vector
            link.g_a(1:3,4,frmIdx(iFrm)) = ...
                (link.m_a(frmIdx(iFrm)) * link.g_a(1:3,4,frmIdx(iFrm)) ...
                + xCoM_i * m_i ) / (link.m_a(frmIdx(iFrm)) + m_i);

            % Assign masses
            link.m_a(frmIdx(iFrm)) = link.m_a(frmIdx(iFrm)) + m_i;

            % Compute inertia tensors w.r.t. frame origins

            % We assume point masses for simplicity
            J_i = m_i * (norm(xCoM_i)^2 * eye(3) - xCoM_i*xCoM_i.');


            MGen_i = blkdiag(J_i, eye(3) * m_i);
            link.M_a(:,:,frmIdx(iFrm)) = link.M_a(:,:,frmIdx(iFrm)) + MGen_i;
        end
    end

    % Check whether the added mass components correspond to the true
    % physical mass
    disp("Overall external mass in LinkDef:");
    disp(sum(link.m_a));

    disp("Mass of individual components:");
    disp(13*pars.StandardDisk.m + pars.TipDisk.m + sum(pars.IMUCables.m) + sum(pars.IMUs.m));


    %% Define TCP
    link.hasTCP = true;
    link.g_B_TCP = SE3Matrix(eye(3), [0,0,0.01]);

end