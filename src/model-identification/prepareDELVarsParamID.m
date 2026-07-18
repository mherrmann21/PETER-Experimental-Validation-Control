function [c_DEL, y] = prepareDELVarsParamID(IDSystemNLP, simPars, ...
        q, u, h, qDot0, paramVecSym, yStepIndices, ...
        opts)
    %% Build DEL Constraints and System Outputs for Parameter identification
    arguments
        % System and output definition struct with NLP variables
        IDSystemNLP     (1,1) struct

        % Variables for the NLP dynamics
        simPars         (1,1) MBSimPars
        q               (:,:)
        u               (:,:)
        h               (1,1)
        qDot0           (:,1)

        % Vector of symbolic NLP parameters
        paramVecSym     (:,1)

        % Time step indices, at which the output is computed
        yStepIndices    (:,1) double {mustBeInteger}

        % Whether to use a CasADi function map to compute the step data or
        % use a simple for loop
        opts.useFunctionMap (1,1) logical = true;
    end

    % Get system objects
    MBSys    = IDSystemNLP.MBSys;
    IMUDef   = IDSystemNLP.IMUDef;
    cableDef = IDSystemNLP.cableDef;

    nSteps = round( simPars.tEnd / h );

    % Weighting factor for generalized trapezoidal rule
    % Rectangle rule: a = 0, trapezoidal rule: a = 1/2
    aFirst = 1/2;
    aStep = 0*1/2;


    %% Define Function for step constraints and outputs

    q_k0Sym = casadi.MX.sym('q_k0', MBSys.nDoF, 1);
    q_kSym  = casadi.MX.sym('q_k', MBSys.nDoF, 1);
    q_k1Sym = casadi.MX.sym('q_k1', MBSys.nDoF, 1);
    u_kSym  = casadi.MX.sym('u_k', MBSys.nInputs, 1);

    % rCalibSym = casadi.MX.sym('r_calib', 3, IMUDef.nIMUs);
    % xCalibSym = casadi.MX.sym('x_calib', 2, IMUDef.nIMUs);
    % IMUDef.rCalib = rCalibSym;
    % IMUDef.xCalib = xCalibSym;
    %IMUDef.rCalib = IDVars.rCalib.NLPVar;
    %IMUDef.xCalib = IDVars.xCalib.NLPVar;

    [g_kSym,  g_rel_kSym]  = MBSys.computeFwdKin(q_kSym);
    [g_k0Sym, g_rel_k0Sym] = MBSys.computeFwdKin(q_k0Sym);
    [g_k1Sym,   g_rel_k1Sym] = MBSys.computeFwdKin(q_k1Sym);

    eta_k0Sym = MBSys.computeDiscreteAbsoluteVelocities(g_rel_k0Sym, g_rel_kSym, h);
    eta_kSym  = MBSys.computeDiscreteAbsoluteVelocities(g_rel_kSym,  g_rel_k1Sym, h);

    % External frame forces from the environment
    f_frame_k_b_ext = zeros(6, MBSys.nFrames);
    f_frame_k_s_ext = zeros(6, MBSys.nFrames);

    % DEL residuum
    DEL_res_k_sym = computeDELResiduum_casadi_extKin(MBSys, simPars, ...
        q_k0Sym, q_kSym, q_k1Sym, g_kSym, g_rel_kSym, eta_kSym, eta_k0Sym, ...
        u_kSym, f_frame_k_b_ext, f_frame_k_s_ext, h, aStep);

    % System outputs
    [y_IMU_gyr, y_IMU_acc] = computeIMUSystemOutput_casadi(MBSys, IMUDef,...
        q_k0Sym, q_kSym, q_k1Sym, g_kSym, g_k0Sym, g_k1Sym, h);

    yLc = computeTendonLengthSystemOutput_casadi(MBSys, cableDef,...
        q_kSym, g_rel_kSym);

    stepFun = casadi.Function('FStep', ...
        {q_k0Sym, q_kSym, q_k1Sym, u_kSym, paramVecSym}, ...
        [{DEL_res_k_sym}; y_IMU_gyr; y_IMU_acc; {yLc}]);


    %% Initial step

    [g_k, g_rel_k] = MBSys.computeFwdKin(q(:,1));
    [~ , g_rel_k1] = MBSys.computeFwdKin(q(:,2));
    eta_k = MBSys.computeDiscreteAbsoluteVelocities(g_rel_k, g_rel_k1, h);

    % TODO: Compute for non-zero p0
    %p0 = zeros(size(OCP.q0));
    c_DEL_1 = computeDELResiduumFirstStep_casadi_extKin( MBSys, simPars, ...
        q(:,1), q(:,2), g_k, g_rel_k, eta_k, u(:,1), qDot0, ...
        h, aFirst);

    % System output not necessary at initial step; outputs start at k=2 or
    % later


    %% Intermediate steps k = 1, ..., N-1 (indices 2, ..., nSteps)

    y = struct;

    if opts.useFunctionMap

        stepFunMap = stepFun.map(nSteps-1, 'thread', 8);

        [cMap_DEL, ...
            yMapIMUGyr1, yMapIMUGyr2, ...
            yMapIMUAcc1, yMapIMUAcc2, ...
            yMapLc] = stepFunMap( ...
            q(:,1:nSteps-1), q(:,2:nSteps), q(:,3:nSteps+1), u(:,2:nSteps), ...
            repmat(paramVecSym, [1, nSteps-1]) ...
            );

        c_DEL = horzcat(c_DEL_1, cMap_DEL);

        % Subtract one from the stepIndices vector since the map output
        % starts at k = 2
        y.IMUGyr1 = yMapIMUGyr1(:,yStepIndices-1);
        y.IMUGyr2 = yMapIMUGyr2(:,yStepIndices-1);
        y.IMUAcc1 = yMapIMUAcc1(:,yStepIndices-1);
        y.IMUAcc2 = yMapIMUAcc2(:,yStepIndices-1);
        y.Lc      = yMapLc(:,yStepIndices-1);

    else

        % Initialize output arrays
        c_DEL = cell(nSteps,1);

        y.IMUGyr1 = cell(nSteps+1,1);
        y.IMUGyr2 = cell(nSteps+1,1);
        y.IMUAcc1 = cell(nSteps+1,1);
        y.IMUAcc2 = cell(nSteps+1,1);
        y.Lc      = cell(nSteps+1,1);

        for k = 2:nSteps
            % [c_DEL{k}, ...
            %     y.IMUGyr1{k}, y.IMUGyr2{k}, ...
            %     y.IMUAcc1{k}, y.IMUAcc2{k}, ...
            %     y.Lc{k}] = stepFun(q(:,k-1), q(:,k), q(:,k+1), u(:,k), paramVecSym);

            q_k  = q(:,k);
            q_k0 = q(:,k-1);
            q_k1 = q(:,k+1);
            u_k  = u(:,k);
            [g_k,  g_rel_k]  = MBSys.computeFwdKin(q_k);
            [g_k0, g_rel_k0] = MBSys.computeFwdKin(q_k0);
            [g_k1, g_rel_k1] = MBSys.computeFwdKin(q_k1);

            eta_k0 = MBSys.computeDiscreteAbsoluteVelocities(g_rel_k0, g_rel_k, h);
            eta_k  = MBSys.computeDiscreteAbsoluteVelocities(g_rel_k,  g_rel_k1, h);

            % External frame forces from the environment
            f_frame_k_b_ext = zeros(6, MBSys.nFrames);
            f_frame_k_s_ext = zeros(6, MBSys.nFrames);

            % DEL residuum
            c_DEL{k} = computeDELResiduum_casadi_extKin(MBSys, simPars, ...
                q_k0, q_k, q_k1, g_k, g_rel_k, eta_k, eta_k0, ...
                u_k, f_frame_k_b_ext, f_frame_k_s_ext, h, aStep);

            % System outputs
            [y_IMU_gyr, y_IMU_acc] = computeIMUSystemOutput_casadi(MBSys, IMUDef,...
                q_k0, q_k, q_k1, g_k, g_k0, g_k1, h);

            y.IMUGyr1{k} = y_IMU_gyr{1};
            y.IMUGyr2{k} = y_IMU_gyr{2};
            y.IMUAcc1{k} = y_IMU_acc{1};
            y.IMUAcc2{k} = y_IMU_acc{2};

            y.Lc{k} = computeTendonLengthSystemOutput_casadi(MBSys, cableDef,...
                q_k, g_rel_k);
        end
        c_DEL = horzcat(c_DEL_1, c_DEL{:});
        y.IMUGyr1 = horzcat(y.IMUGyr1{yStepIndices});
        y.IMUGyr2 = horzcat(y.IMUGyr2{yStepIndices});
        y.IMUAcc1 = horzcat(y.IMUAcc1{yStepIndices});
        y.IMUAcc2 = horzcat(y.IMUAcc2{yStepIndices});
        y.Lc      = horzcat(y.Lc{yStepIndices});
    end
end
