function [qSim, ySim, LtOffsetSim] = computeSetPointEqulibria(MBSim, uSP, IMUDef, tendonDef)
    %% Compute static equilibria and system outputs for set of tendon tensions
    arguments
        MBSim       (1,1) MBSimulation
        uSP         (:,:) double
        IMUDef      (1,1) MBSysIMUOutputDefinition
        tendonDef   (1,1) MBSysTendonLengthOutputDefinition
    end

    nSetpoints = size(uSP,2);

    ySim = struct;
    ySim.Lt  = zeros(tendonDef.nTendons, nSetpoints);
    nIMUs = length(IMUDef.s);
    ySim.Acc = zeros(3, nIMUs, nSetpoints);

    opts = optimoptions('fsolve', ...
        'Display','none', ...
        'Algorithm','trust-region', ...
        'FunctionTolerance', 1e-10, ...
        'OptimalityTolerance', 1e-10);

    qSim = zeros(MBSim.MBSys.nDoF, nSetpoints);
    q0 = zeros(MBSim.MBSys.nDoF,1);

    fprintf("\nComputing equilibrium XX / YY...");
    for iSP = 1:nSetpoints
        fprintf("\b\b\b\b\b\b\b\b\b\b%2d / %2d...", iSP, nSetpoints);

        [qEqu, fval, exitflag, output] = fsolve( ...
            @(q) computeStaticResiduum_mex(MBSim.MBSys, MBSim.simPars, q, uSP(:,iSP)), ...
            q0, opts ...
            );
        if exitflag < 1
            warning("Solution failed.");
        end
        qSim(:,iSP) = qEqu;
        q0 = qEqu;

        % Compute outputs
        gEqu = MBSim.MBSys.computeFwdKin(qEqu);

        [~, ySim.Acc(:,:,iSP)] = computeIMUSystemOutput( ...
            MBSim.MBSys, IMUDef, ...
            qEqu, qEqu, qEqu, gEqu, gEqu, gEqu, 1);

        ySim.Lt(:,iSP) = computeTendonLengthSystemOutput( ...
            MBSim.MBSys, tendonDef, qEqu);
    end
    fprintf("\n");


    % Tendon length offset with zero inputs
    disp("Computing reference equilibrium (zero inputs)...")
    [qEqu_u0, fval, exitflag, output] = fsolve( ...
        @(q) computeStaticResiduum(MBSim.MBSys, MBSim.simPars, q, zeros(MBSim.MBSys.nInputs,1)), ...
        zeros(MBSim.MBSys.nDoF,1), opts ...
        );
    LtOffsetSim = computeTendonLengthSystemOutput( ...
        MBSim.MBSys, tendonDef, qEqu_u0);

    %LtOffsetSim = ySim.Lt(:,1)*0+ links.L;
    %ySim.Lt = ySim.Lt - LtOffsetSim;
end
