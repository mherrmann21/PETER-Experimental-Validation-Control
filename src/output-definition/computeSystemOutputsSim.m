function [y, LcOffset]  = computeSystemOutputsSim(MBSim, IMUDef, cableDef, opts)
    %% Compute system outputs for a full simulation
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        % MBSimulation containing the results data
        MBSim           (1,1) MBSimulation

        % IMU definition
        IMUDef          (1,1) MBSysIMUOutputDefinition

        % Cable output definition
        cableDef        (1,1) MBSysTendonLengthOutputDefinition

        % Wether to use the absolute tendon length (= false) or the offset
        % from the initial length as the tendon length measurement/output
        opts.useTendonLengthOffset (1,1) logical = true;
    end

    nIMUs   = length(IMUDef.s);
    nCables = cableDef.nCables;
    nSteps  = length(MBSim.simRes.tout)-1;

    yAll    = zeros(6*nIMUs+nCables, nSteps);
    yIMUAcc = zeros(3, nIMUs, nSteps);
    yIMUGyr = zeros(3, nIMUs, nSteps);
    yLc     = zeros(nCables, nSteps);

    for iStep = 1:nSteps
        %eta_k = MBSim.simRes.eta(:,:,iStep);
        g_k   = MBSim.simRes.g(:,:,:,iStep);
        g_k1  = MBSim.simRes.g(:,:,:,iStep+1);
        q_k   = MBSim.simRes.q(:,iStep);
        q_k1  = MBSim.simRes.q(:,iStep+1);
        if iStep == 1
            %eta_k0 = eta_k;
            g_k0   = g_k;
            q_k0   = q_k;
        else
            %eta_k0 = MBSim.simRes.eta(:,:,iStep-1);
            g_k0  = MBSim.simRes.g(:,:,:,iStep-1);
            q_k0  = MBSim.simRes.q(:,iStep-1);
        end
        hStep = MBSim.simRes.tout(iStep+1)-MBSim.simRes.tout(iStep);
        [yIMUGyr(:,:,iStep), yIMUAcc(:,:,iStep)] = computeIMUSystemOutput( ...
            MBSim.MBSys, IMUDef, ...
            q_k, q_k0, q_k1, g_k, g_k0, g_k1, hStep);

        yLc(:,iStep) = computeTendonLengthSystemOutput( ...
            MBSim.MBSys, cableDef, q_k);
    end

    % Compute offset from initial length
    LcOffset = yLc(:,1);
    if opts.useTendonLengthOffset
        yLc = yLc - LcOffset;
    end

    % Assign to output struct
    y = struct();
    y.yAll = yAll;
    y.IMUAcc = yIMUAcc;
    y.IMUGyr = yIMUGyr;
    y.Lc     = yLc;
    y.tout = MBSim.simRes.tout(1:end-1);
end