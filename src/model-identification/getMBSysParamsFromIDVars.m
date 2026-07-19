function IDSystemOpt = getMBSysParamsFromIDVars(IDSystemNum, IDSystemNLP, sol)
    %% Get numeric MBSystem Object from NLP solution
    arguments (Input)
        % System and output definition struct with nominal parameters
        IDSystemNum (1,1) struct

        % System and output definition struct with NLP variables
        IDSystemNLP (1,1) struct

        % CasADi Opti solution object containing the identified values
        sol         (1,1) casadi.OptiSol
    end

    %% Get system objects

    MBSysNum    = IDSystemNum.MBSys;
    IMUDefNum   = IDSystemNum.IMUDef;
    tendonDefNum = IDSystemNum.tendonDef;

    MBSysNLP    = IDSystemNLP.MBSys;
    IMUDefNLP   = IDSystemNLP.IMUDef;
    tendonDefNLP = IDSystemNLP.tendonDef;


    %% Get parameters

    MBSysNum.cSys = sol.value(MBSysNLP.cSys);

    MBSysNum.dSys = sol.value(MBSysNLP.dSys);

    MBSysNum.frameData.m = sol.value(MBSysNLP.frameData.m);

    MBSysNum.frameData.m_a = sol.value(MBSysNLP.frameData.m_a);

    MBSysNum.frameData.x_a = sol.value(MBSysNLP.frameData.x_a);

    % Generalized frame inertia
    for iFrm = 1:MBSysNum.nFrames
        MBSysNum.frameData.MGen(:,:,iFrm) = sol.value(MBSysNLP.frameData.MGen{iFrm});
    end

    % IMU configuration
    for iIMU = 1:size(IMUDefNum.g_rel, 3)
        IMUDefNum.g_rel(:,:,iIMU) = sol.value(IMUDefNLP.g_rel(iIMU).mat);
    end

    % Tendon path configuration
    for iT = 1:MBSysNum.nInputs
        % Tendon path in framedata
        for iSeg = 1:size(MBSysNLP.frameData.g_cm,2)
            MBSysNum.frameData.g_cm(:,:,1,iSeg,iT) = sol.value(MBSysNLP.frameData.g_cm(1,iSeg,iT).mat);
            MBSysNum.frameData.g_cm(:,:,2,iSeg,iT) = sol.value(MBSysNLP.frameData.g_cm(2,iSeg,iT).mat);
        end

        % Tendon path in tendonDef
        for iSeg = 1:size(tendonDefNLP.g_tm_SE3,2)
            tendonDefNum.g_tm_Mat(:,:,1,iSeg,iT) = sol.value(tendonDefNLP.g_tm_SE3(1,iSeg,iT).mat);
            tendonDefNum.g_tm_Mat(:,:,2,iSeg,iT) = sol.value(tendonDefNLP.g_tm_SE3(2,iSeg,iT).mat);
        end
    end

    %MBSysNum.frameData.d_c_s = sol.value(MBSysID.frameData.d_c_s);

    IDSystemOpt = struct;
    IDSystemOpt.MBSys    = MBSysNum;
    IDSystemOpt.IMUDef   = IMUDefNum;
    IDSystemOpt.tendonDef = tendonDefNum;
end
