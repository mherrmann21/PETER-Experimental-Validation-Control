function [y, LtOffset]  = computeSystemOutputsSim(MBSim, IMUDef, tendonDef, opts)
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

        % Tendon output definition
        tendonDef       (1,1) MBSysTendonLengthOutputDefinition

        % Whether to use the absolute tendon length (= false) or the offset
        % from the initial length as the tendon length measurement/output
        opts.useTendonLengthOffset (1,1) logical = true;
    end

    nIMUs   = length(IMUDef.s);
    nTendons = tendonDef.nTendons;
    nSteps  = length(MBSim.simRes.tout)-1;

    yIMUAcc = zeros(3, nIMUs, nSteps);
    yIMUGyr = zeros(3, nIMUs, nSteps);
    yLt     = zeros(nTendons, nSteps);

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

        yLt(:,iStep) = computeTendonLengthSystemOutput( ...
            MBSim.MBSys, tendonDef, q_k);
    end

    % Compute offset from initial length
    LtOffset = yLt(:,1);
    if opts.useTendonLengthOffset
        yLt = yLt - LtOffset;
    end

    % Assign to output struct
    y = struct();
    y.IMUAcc = yIMUAcc;
    y.IMUGyr = yIMUGyr;
    y.Lt     = yLt;
    y.tout = MBSim.simRes.tout(1:end-1);
end
