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
    cableDefNum = IDSystemNum.cableDef;

    MBSysNLP    = IDSystemNLP.MBSys;
    IMUDefNLP   = IDSystemNLP.IMUDef;
    cableDefNLP = IDSystemNLP.cableDef;


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
    for iC = 1:MBSysNum.nInputs
        % Tendon path in framedata
        for iSeg = 1:size(MBSysNLP.frameData.g_cm,2)
            MBSysNum.frameData.g_cm(:,:,1,iSeg,iC) = sol.value(MBSysNLP.frameData.g_cm(1,iSeg,iC).mat);
            MBSysNum.frameData.g_cm(:,:,2,iSeg,iC) = sol.value(MBSysNLP.frameData.g_cm(2,iSeg,iC).mat);
        end

        % Tendon path in cabledef
        for iSeg = 1:size(cableDefNLP.g_cm_SE3,2)
            cableDefNum.g_cm_Mat(:,:,1,iSeg,iC) = sol.value(cableDefNLP.g_cm_SE3(1,iSeg,iC).mat);
            cableDefNum.g_cm_Mat(:,:,2,iSeg,iC) = sol.value(cableDefNLP.g_cm_SE3(2,iSeg,iC).mat);
        end
    end

    %MBSysNum.frameData.d_c_s = sol.value(MBSysID.frameData.d_c_s);

    IDSystemOpt = struct;
    IDSystemOpt.MBSys    = MBSysNum;
    IDSystemOpt.IMUDef   = IMUDefNum;
    IDSystemOpt.cableDef = cableDefNum;
end