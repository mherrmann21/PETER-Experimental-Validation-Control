function [c_Equ, y] = prepareStaticEquVarsParamID(IDSystemNLP, simPars, ...
        q, u, paramVecSym, ...
        opts)
    %% Build DEL Constraints and System Outputs for Parameter identification
    arguments
        % System and output definition struct with NLP variables
        IDSystemNLP

        % Variables for the NLP statics
        simPars         (1,1) MBSimPars
        q               (:,:)
        u               (:,:)

        % Vector of symbolic NLP parameters
        paramVecSym     (:,1)

        % Wether to use a casadi function map to compute the step data or
        % use a simple for loop
        opts.useFunctionMap (1,1) logical = true;
    end

    % Get system objects
    MBSys    = IDSystemNLP.MBSys;
    IMUDef   = IDSystemNLP.IMUDef;
    cableDef = IDSystemNLP.cableDef;

    nSetpoints = size(u, 2);

    %% Define Function for equ. constraints and outputs

    qSym = casadi.MX.sym('q', MBSys.nDoF, 1);
    uSym  = casadi.MX.sym('u', MBSys.nInputs, 1);

    [res_sym, gSym, g_rel_Sym] = computeStaticResiduum_casadi(MBSys, simPars, qSym, uSym);


    % System outputs
    h = 1;
    [~, y_IMU_acc] = computeIMUSystemOutput_casadi(MBSys, IMUDef,...
        qSym, qSym, qSym, gSym, gSym, gSym, h);

    yLc = computeTendonLengthSystemOutput_casadi(MBSys, cableDef,...
        qSym, g_rel_Sym);

    setPointFun = casadi.Function('FSP', ...
        {qSym, uSym, paramVecSym}, ...
        [{res_sym}; y_IMU_acc; {yLc}]);


    %% Map for all setpoints

    y = struct;

    if opts.useFunctionMap

        setPointMap = setPointFun.map(nSetpoints, 'thread', 8);

        [cMap_Equ,  yMapIMUAcc1, yMapIMUAcc2, yMapLc] = setPointMap( ...
            q, u, repmat(paramVecSym, [1, nSetpoints]) ...
            );

        c_Equ = cMap_Equ;
        y.IMUAcc1 = yMapIMUAcc1;
        y.IMUAcc2 = yMapIMUAcc2;
        y.Lc      = yMapLc;

    else

        %%% TODO verify

        % Initialize output arrays
        c_Equ = cell(nSetpoints,1);

        y.IMUAcc1 = cell(nSetpoints,1);
        y.IMUAcc2 = cell(nSetpoints,1);
        y.Lc      = cell(nSetpoints,1);

        for k = 1:nSetpoints
            [c_Equ{k}, y.IMUAcc1{k}, y.IMUAcc2{k},  y.Lc{k}] = setPointFun( ...
                q(:,k), u(:,k), paramVecSym);
        end
        c_Equ = horzcat(c_Equ{:});
        y.IMUAcc1 = horzcat(y.IMUAcc1{:});
        y.IMUAcc2 = horzcat(y.IMUAcc2{:});
        y.Lc      = horzcat(y.Lc{:});
    end
end
