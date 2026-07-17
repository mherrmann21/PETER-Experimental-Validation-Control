function [IDSystemNLP, IDVars, pVectors] = getParamIDMBSys( ...
        opti, includeDynamic, calibrationOnly, IDSystemSym, LcOffsetSim)
    %% Get MBSystem parameterized with symbolic NLP variables
    % and the corresponding auxiliary variables
    arguments (Input)
        opti            (1,1) casadi.Opti

        % Whether to include the parameters for dynamics
        % (dissipation and inertia tensors)
        includeDynamic  (1,1) logical

        % Calibration only -> just kinematic params + dissipation
        calibrationOnly (1,1) logical

        % System and output definition struct with nominal parameters
        IDSystemSym     (1,1) struct

        % Cable length offset used as initial values for Lc
        LcOffsetSim     (:,1) double
    end

    MBSysSym = IDSystemSym.MBSys;
    IMUDef   = IDSystemSym.IMUDef;
    cableDef = IDSystemSym.cableDef;

    IDVars = struct();
    % IDVars.X.NLPVar   NLP variable for the parameter
    % IDVars.X.lb       NLP var. lower bound
    % IDVars.X.ub       NLP var. upper bound
    % IDVars.X.iv       NLP var. initial value
    % IDVars.X.nv       Nominal (absolute) value for the system parameter
    % IDVars.X.fr       Regularization factors (same dimension as NLPVar,
    %                   default value 1)

    MBSysNLP = MBSysSym;

    if ~calibrationOnly
        % Stiffness
        % Common multiplier and small element-wise factor
        IDVars.cSysC.NLPVar = opti.variable(1);
        IDVars.cSysC.lb = 0.5;
        IDVars.cSysC.ub = 1.5;
        IDVars.cSysC.iv = 1;
        IDVars.cSysC.fr = 1;
        IDVars.cSysC.nv = 1;

        IDVars.cSysE.NLPVar = opti.variable(size(MBSysSym.cSys, 1));
        IDVars.cSysE.lb = 0.99;
        IDVars.cSysE.ub = 1.01;
        IDVars.cSysE.iv = ones(size(MBSysSym.cSys));
        IDVars.cSysE.fr = ones(size(MBSysSym.cSys));
        IDVars.cSysE.nv = MBSysSym.cSys;

        MBSysNLP.cSys = MBSysSym.cSys .* IDVars.cSysE.NLPVar * IDVars.cSysC.NLPVar;
    end

    % Dissipation
    if includeDynamic
        IDVars.dSysC.NLPVar = opti.variable(1);
        IDVars.dSysC.lb = 1e-3;
        IDVars.dSysC.ub = 1e3;
        IDVars.dSysC.iv = 1;
        IDVars.dSysC.fr = 1e-2;
        IDVars.dSysC.nv = 1;

        IDVars.dSysE.NLPVar = opti.variable(size(MBSysSym.dSys, 1));
        IDVars.dSysE.lb = 0.8;
        IDVars.dSysE.ub = 1.2;
        IDVars.dSysE.iv = ones(size(MBSysSym.dSys));
        IDVars.dSysE.fr = ones(size(MBSysSym.dSys))*1e-1;
        IDVars.dSysE.nv = MBSysSym.dSys;
        MBSysNLP.dSys = MBSysSym.dSys .* IDVars.dSysE.NLPVar * IDVars.dSysC.NLPVar;
    end


    % Frame masses
    if ~calibrationOnly
        IDVars.mC.NLPVar = opti.variable(1);
        IDVars.mC.lb = 0.5;
        IDVars.mC.ub = 2;
        IDVars.mC.iv = 1;
        IDVars.mC.fr = 1;
        IDVars.mC.nv = 1;

        IDVars.mE.NLPVar = opti.variable(size(MBSysSym.frameData.m, 1));
        IDVars.mE.lb = 0.25;
        IDVars.mE.ub = 4;
        IDVars.mE.iv = ones(size(MBSysSym.frameData.m));
        IDVars.mE.fr = ones(size(MBSysSym.frameData.m));
        IDVars.mE.nv = MBSysSym.frameData.m;

        MBSysNLP.frameData.m = IDVars.mC.NLPVar *IDVars.mE.NLPVar .* MBSysSym.frameData.m;
    end
    % % Static tendon friction
    % IDVars.d_c_s.NLPVar = opti.variable(1);
    % IDVars.d_c_s.lb = 1e-3;
    % IDVars.d_c_s.ub = 1e3;
    % IDVars.d_c_s.iv = 1;
    % IDVars.d_c_s.nv = MBSysSym.frameData.d_c_s;
    % MBSysNLP.frameData.d_c_s = MBSysSym.frameData.d_c_s .* IDVars.d_c_s.NLPVar.';



    %% Frame inertia
    USE_SIMPLE_INERTIA_IDENT = true;

    if includeDynamic && ~calibrationOnly
        if USE_SIMPLE_INERTIA_IDENT
            % Only use scalar factors for inertia tensors

            % IDVars.l.NLPVar = opti.variable(MBSysSym.nFrames,1);
            % IDVars.l.lb = 1e-3;
            % IDVars.l.ub = 1e3;
            % IDVars.l.iv = ones(MBSysSym.nFrames,1);
            % IDVars.l.fr = ones(MBSysSym.nFrames,1);
            % IDVars.l.nv = 1;

            for iFrm = 1:MBSysSym.nFrames
                %J_i = MBSysSym.frameData.MGen{iFrm}(1:3,1:3) * IDVars.l.NLPVar(iFrm);
                J_i = MBSysSym.frameData.MGen{iFrm}(1:3,1:3) * IDVars.mC.NLPVar * IDVars.mE.NLPVar(iFrm);
                M_i = casadi.MX.eye(3)*MBSysNLP.frameData.m(iFrm);

                % Generalized inertia matrix
                MBSysNLP.frameData.MGen{iFrm} = blkdiag(J_i, M_i);
            end

        else
            % Nominal cholesky parameters of the inertia tensors
            L_Nom = zeros(3,3, MBSysSym.nFrames);
            lVecNom = zeros(6, MBSysSym.nFrames);
            for iFrm = 1:MBSysSym.nFrames
                L_Nom(:,:,iFrm) = chol(MBSysSym.frameData.MGen(1:3,1:3,iFrm), "lower");
                LNom_i = L_Nom(:,:,iFrm);
                lVecNom(:,iFrm) = [LNom_i(1,1), LNom_i(2,2), LNom_i(3,3), LNom_i(2,1), LNom_i(3,1), LNom_i(3,2)];
            end

            %%% Define inertia tensors from NLP cholesky parameters

            % Get scaling vector; set zero values to mean value
            lScale = lVecNom;
            lScale(abs(lScale) < 1e-8 ) = mean(lScale(abs(lScale)>1e-8), "all");

            IDVars.l.NLPVar = opti.variable(6, MBSysSym.nFrames);

            % Bounds for inertia tensor: diagonal elements of L must be positive
            IDVars.l.lb = [ones(3, MBSysSym.nFrames)*1e-9; ones(3, MBSysSym.nFrames)*-1e2];
            IDVars.l.ub = ones(6, MBSysSym.nFrames) * 1e2;
            IDVars.l.iv = lVecNom./lScale;
            IDVars.l.fr = ones(6, MBSysSym.nFrames);
            IDVars.l.nv = lVecNom;

            for iFrm = 1:MBSysSym.nFrames
                % Vector of symbolic variables in the triangular matrix
                l = IDVars.l.NLPVar(:,iFrm);

                % Construct lower triangular matrix L
                ls = l .* lScale(:,iFrm); % Scaled parameters
                L = [ls(1), 0, 0
                    ls(4),ls(2),0
                    ls(5),ls(6),ls(3)
                    ];

                % Compute symmetric, positive-definite inertia tensor from L
                J_i = L * L.';

                % Generalized inertia matrix
                MBSysNLP.frameData.MGen{iFrm} = blkdiag(J_i, casadi.MX.eye(3)*MBSysNLP.frameData.m(iFrm));
            end
        end
    end

    %% External masses

    extMassFrames = find(MBSysSym.frameData.m_a);
    nExtMasses = sum(MBSysSym.frameData.m_a ~= 0);

    % Masses (as factors)
    if ~calibrationOnly
        IDVars.m_a_C.NLPVar = opti.variable(1);
        IDVars.m_a_C.lb = 1;0.25;1e-3;
        IDVars.m_a_C.ub = 1;4;1e3;
        IDVars.m_a_C.iv = 1;
        IDVars.m_a_C.fr = 1;
        IDVars.m_a_C.nv = 1;

        IDVars.m_a_E.NLPVar = opti.variable(nExtMasses, 1);
        IDVars.m_a_E.lb = 1;0.5;0.1;
        IDVars.m_a_E.ub = 1;2;10;
        IDVars.m_a_E.iv = ones(nExtMasses, 1);
        IDVars.m_a_E.fr = ones(nExtMasses, 1);
        IDVars.m_a_E.nv = MBSysSym.frameData.m_a(extMassFrames);


        MBSysNLP.frameData.m_a = casadi.MX.zeros(1,MBSysSym.nFrames);
        MBSysNLP.frameData.m_a(extMassFrames) = ...
            IDVars.m_a_C.NLPVar * MBSysSym.frameData.m_a(extMassFrames).' .* IDVars.m_a_E.NLPVar;


        % CoM position vectors (as factors)
        if 0
            IDVars.x_a.NLPVar = opti.variable(3, nExtMasses);
            IDVars.x_a.lb = -2;
            IDVars.x_a.ub = +2;
            IDVars.x_a.iv = ones(3,nExtMasses);
            IDVars.x_a.fr = ones(3,nExtMasses);
            IDVars.x_a.nv = MBSysSym.frameData.x_a(:,extMassFrames);
            MBSysNLP.frameData.x_a = casadi.MX.zeros(3,MBSysSym.nFrames);
            MBSysNLP.frameData.x_a(:,extMassFrames) = ...
                MBSysSym.frameData.x_a(:,extMassFrames) .* IDVars.x_a.NLPVar;

        else
            % CoM position vectors (as polar coordinates)
            IDVars.phi_x_a.NLPVar = opti.variable(nExtMasses);
            IDVars.phi_x_a.lb = -deg2rad(70);
            IDVars.phi_x_a.ub = +deg2rad(70);
            IDVars.phi_x_a.iv = zeros(nExtMasses,1)+deg2rad(40);
            IDVars.phi_x_a.fr = ones(nExtMasses,1);
            IDVars.phi_x_a.nv = MBSysSym.frameData.x_a(:,extMassFrames);

            IDVars.r_x_a.NLPVar = opti.variable(nExtMasses);
            IDVars.r_x_a.lb = 0.5;
            IDVars.r_x_a.ub = 2;
            IDVars.r_x_a.iv = ones(nExtMasses,1);
            IDVars.r_x_a.fr = ones(nExtMasses,1);
            IDVars.r_x_a.nv = MBSysSym.frameData.x_a(:,extMassFrames);


            MBSysNLP.frameData.x_a = casadi.MX.zeros(3,MBSysSym.nFrames);

            for iM = 1:nExtMasses
                phi_m = IDVars.phi_x_a.NLPVar(iM);%*pi/180;
                R_m_rel = [
                    cos(phi_m), -sin(phi_m), 0;
                    sin(phi_m),  cos(phi_m), 0;
                    0         , 0          , 1];

                MBSysNLP.frameData.x_a(:,extMassFrames(iM)) = ...
                    IDVars.r_x_a.NLPVar(extMassFrames(iM)) * R_m_rel * MBSysSym.frameData.x_a(:,extMassFrames(iM));
            end
        end

    end


    %% IMU calibration variables

    nIMUs = length(IMUDef.s);

    % IMU radius (factor)
    IDVars.rIMU.NLPVar = opti.variable(nIMUs,1);
    IDVars.rIMU.lb = 0.98;
    IDVars.rIMU.ub = 1.02;
    IDVars.rIMU.iv = ones(nIMUs,1);
    IDVars.rIMU.fr = ones(nIMUs,1) * 1;
    IDVars.rIMU.nv = 1;

    % IMU polar angle offset (deg)
    IDVars.phiIMU.NLPVar = opti.variable(nIMUs,1);
    IDVars.phiIMU.lb = -7;
    IDVars.phiIMU.ub = +7;
    IDVars.phiIMU.iv = zeros(nIMUs,1);
    IDVars.phiIMU.fr = ones(nIMUs,1) * 0.01;
    IDVars.phiIMU.nv = 0;

    % Rotation offset in terms of cayley parameters
    IDVars.rCalib.NLPVar = opti.variable(3, nIMUs);
    IDVars.rCalib.lb = -deg2rad(20);
    IDVars.rCalib.ub = +deg2rad(20);
    IDVars.rCalib.iv = zeros(3,nIMUs);
    IDVars.rCalib.fr = ones(3,nIMUs) * 1e-3;
    IDVars.rCalib.nv = zeros(3,nIMUs);

    IMUDefNLP = IMUDef;
    %IMUDef_NLP.rCalib = IDVars.rCalib.NLPVar;
    %IMUDef_NLP.xCalib = IDVars.xCalib.NLPVar;

    % SE3 transformation from cross-section frame to IMU frame
    IMUDefNLP.g_rel = SE3MatArray2SE3Array(IMUDef.g_rel);
    %IMUDefNLP.g_rel = createArray([nIMUs, 1], "SE3");

    fSE3 = getSE3Functions();
    for iIMU = 1:nIMUs
        % Get original transformation with scaled radius
        g_orig = IMUDefNLP.g_rel(iIMU);
        g_orig.x = g_orig.x * IDVars.rIMU.NLPVar(iIMU);

        % Additional transformation about CS z-axis
        phi_IMU = IDVars.phiIMU.NLPVar(iIMU)*pi/180;
        R_IMU_rel = [
            cos(phi_IMU), -sin(phi_IMU), 0;
            sin(phi_IMU),  cos(phi_IMU), 0;
            0           , 0            , 1];
        g_rot_rel = SE3(R_IMU_rel, casadi.MX.zeros(3,1));

        % Additional calibration transformation
        R_IMU_calib = fSE3.caySO3(IDVars.rCalib.NLPVar(:,iIMU));
        g_calib = SE3(R_IMU_calib, casadi.MX.zeros(3,1));

        % Overall IMU configuration (relative to backbone)
        IMUDefNLP.g_rel(iIMU) = g_rot_rel * g_orig * g_calib;

        % Radius to IMU
        %r_IMU = IDVars.rIMU.nv(iIMU) * IDVars.rIMU.NLPVar(iIMU);

        %x_IMU_rel = R_IMU_rel * [r_IMU; 0; 0];

        % Calibration transformation

        % Additional transformation to the IMU measurement frame
        %5R_IMU_rel_fr = R_IMU_rel * IMUParams.Rconst * R_IMU_calib;
    end


    %% NLP variables for tendon length

    IDVars.uScale.NLPVar = opti.variable(1);
    IDVars.uScale.lb = 1;
    IDVars.uScale.ub = 1;
    IDVars.uScale.iv = 1;
    IDVars.uScale.fr = 1;
    IDVars.uScale.nv = 1;

    IDVars.LcScaleP.NLPVar = opti.variable(MBSysSym.nInputs);
    IDVars.LcScaleP.lb = 0.7;
    IDVars.LcScaleP.ub = 1.5;
    IDVars.LcScaleP.iv = ones(MBSysSym.nInputs,1);
    IDVars.LcScaleP.fr = ones(MBSysSym.nInputs,1)*10;
    IDVars.LcScaleP.nv = 1;

    IDVars.LcScaleN.NLPVar = opti.variable(MBSysSym.nInputs);
    IDVars.LcScaleN.lb = 0.7;
    IDVars.LcScaleN.ub = 1.5;
    IDVars.LcScaleN.iv = ones(MBSysSym.nInputs,1);
    IDVars.LcScaleN.fr = ones(MBSysSym.nInputs,1)*10;
    IDVars.LcScaleN.nv = 1;

    IDVars.LcOffset.NLPVar = opti.variable(MBSysSym.nInputs);
    IDVars.LcOffset.lb = 0.65;
    IDVars.LcOffset.ub = 0.75;
    IDVars.LcOffset.iv = LcOffsetSim;
    IDVars.LcOffset.fr = ones(MBSysSym.nInputs,1) * 1e-3;
    IDVars.LcOffset.nv = LcOffsetSim;


    %% Tendon paths

    cableDefNLP = cableDef;

    if ~calibrationOnly

        % Rotation around z-axis (unit deg)
        IDVars.phiT.NLPVar = opti.variable(MBSysSym.nInputs);
        IDVars.phiT.lb = ones(MBSysSym.nInputs,1) * -10;
        IDVars.phiT.ub = ones(MBSysSym.nInputs,1) * +10;
        IDVars.phiT.iv = zeros(MBSysSym.nInputs,1);
        IDVars.phiT.fr = ones(MBSysSym.nInputs,1) * 1;
        IDVars.phiT.nv = 0;

        % Fix first angle to avoid unnecessary free rotations of the full robot
        IDVars.phiT.lb(1) = 0;
        IDVars.phiT.ub(1) = 0;

        % x_m_fun_straight = @(s,d,alpha) [d*cosd(alpha); d*sind(alpha); 0 ];

        for iT = 1:MBSysSym.nInputs
            phi_T = IDVars.phiT.NLPVar(iT)*pi/180;
            R_T_rel = [
                cos(phi_T), -sin(phi_T), 0;
                sin(phi_T),  cos(phi_T), 0;
                0         , 0          , 1];

            % Tendon path in framedata
            for iSeg = 1:size(MBSysSym.frameData.g_cm,2)
                MBSysNLP.frameData.g_cm(1,iSeg,iT).x ...
                    = R_T_rel * MBSysSym.frameData.g_cm(1,iSeg,iT).x;
                MBSysNLP.frameData.g_cm(2,iSeg,iT).x ...
                    = R_T_rel * MBSysSym.frameData.g_cm(2,iSeg,iT).x;
            end

            % Tendon path in cabledef
            for iSeg = 1:size(cableDef.g_cm_SE3,2)
                cableDefNLP.g_cm_SE3(1,iSeg,iT).x = R_T_rel * cableDef.g_cm_SE3(1,iSeg,iT).x;
                cableDefNLP.g_cm_SE3(2,iSeg,iT).x = R_T_rel * cableDef.g_cm_SE3(2,iSeg,iT).x;
            end
        end
    end

    %% Parameter vectors with struct fields from the parameter struct

    IDVarFields = fieldnames(IDVars);
    NLPVarsCell  = cell(length(fieldnames(IDVars)),1);
    IValuesCell  = cell(length(fieldnames(IDVars)),1);
    FRValuesCell = cell(length(fieldnames(IDVars)),1);

    for iVar = 1:length(IDVarFields)
        NLPVarsCell{iVar}  = IDVars.(IDVarFields{iVar}).NLPVar(:);
        IValuesCell{iVar}  = IDVars.(IDVarFields{iVar}).iv(:);
        FRValuesCell{iVar} = IDVars.(IDVarFields{iVar}).fr(:);
    end

    pVectors = struct();
    pVectors.NLPVar = vertcat(NLPVarsCell{:});
    pVectors.iv = vertcat(IValuesCell{:});
    pVectors.fr = vertcat(FRValuesCell{:});


    %% Assign to output struct
    IDSystemNLP = struct;
    IDSystemNLP.MBSys    = MBSysNLP;
    IDSystemNLP.IMUDef   = IMUDefNLP;
    IDSystemNLP.cableDef = cableDefNLP;
end