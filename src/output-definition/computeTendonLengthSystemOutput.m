function yLt = computeTendonLengthSystemOutput(MBSys, tendonDef, q)
    %% Compute the tendon length outputs of a system in one time step
    arguments
        MBSys       (1,1) MBSystem

        tendonDef   (1,1) MBSysTendonLengthOutputDefinition

        % Coordinates
        q         (:,1)
    end


    %% Compute tendon lengths

    %%% TODO: The output tendon length vector currently always has
    %%% dimensions of overall nr. of inputs (possibly including joint
    %%% actuation), so it only properly works with single-link continuum
    %%% manipulators right now

    g_rel_k = MBSys.computeJointTransformations(q);

    if tendonDef.isDiscrete
        yLt = computeTendonLengthsDiscrete(MBSys, tendonDef, q, g_rel_k);
    else
        yLt = computeTendonLengthsContinuous(MBSys, g_rel_k);
    end
end


function l_t_k = computeTendonLengthsContinuous(MBSys, g_rel_k)
    %% Compute tendon lengths assuming continuous routing along the backbone
    arguments
        MBSys   (1,1) MBSystem
        g_rel_k (4,4,:) double
    end
    l_t_k = zeros(MBSys.nInputs, 1);

    for iFrm = 1:MBSys.nFrames
        uIndices = MBSys.frameData.getUIndices(iFrm);

        if MBSys.frameData.jointType(iFrm) == 2
            % Flexible joint (multiple tendon inputs)
            % l = MBSys.frameData.l(iFrm);

            for iT = 1:length(uIndices)
                % Tendon configurations at adjacent nodes
                g_tm_i1 = eye(4);
                g_tm_i2 = eye(4);
                g_tm_i1(1:3,4) = MBSys.frameData.x_cm(:,1,iFrm,iT);
                g_tm_i2(1:3,4) = MBSys.frameData.x_cm(:,2,iFrm,iT);

                % Discrete deformation gradient tendon routing
                % Tangent vector is in elements 4:6
                xi_t = cayInvSE3( g_tm_i1 \ g_rel_k(:,:,iFrm) * g_tm_i2 );

                l_t_k(uIndices(iT)) = l_t_k(uIndices(iT)) + vecnorm( xi_t(4:6,:) );
            end
        end
    end
end

function l_t_k = computeTendonLengthsDiscrete(MBSys, tendonDef, q, g_rel)
    %% Compute tendon lengths assuming routing via discrete spacer disks
    % adapted code from Leander Pfeiffer
    arguments
        MBSys   (1,1) MBSystem

        tendonDef (1,1) MBSysTendonLengthOutputDefinition

        q       (:,1) double
        g_rel   (4,4,:) double
    end

    nDisks = tendonDef.nDisks;
    nTendons = tendonDef.nTendons;
    l_t_k = zeros(nTendons, 1);

    % Check whether the beam discretization corresponds to spacer disk
    % distribution or not
    if (nDisks-1) == MBSys.nFrames
        % Spacer disks are directly attached to the frames
        % No interpolation necessary
        g_disks_rel = g_rel;
    else
        % Spacer disks may lie between segments
        % Configurations must be interpolated

        % Positions of the beam frames (including first fixed node)
        sFrames = [0; cumsum(MBSys.frameData.l)];

        % Positions of the spacer disks (including first fixed disk)
        sDisks = tendonDef.sDisks;
        lDisks = diff(sDisks);

        % Discrete deformations
        xi = MBSys.getLinkDeformations(q, 1);

        g_disks_rel = zeros(4,4,nDisks);
        for iDiskSeg = 1:(nDisks-1)
            % Get frame before the spacer disk at the end of current SD
            % segment
            % Include some numerical tolerance for robustness
            iFrmSD = find((sDisks(iDiskSeg+1) - sFrames) > 1e-14, 1, "last");

            % Transformation w.r.t. previous spacer disk center
            g_disks_rel(:,:,iDiskSeg) = caySE3(xi(:,iFrmSD)*lDisks(iDiskSeg));
        end
    end
    DEBUG = 0; % Draw tendon contact positions in a plot?
    if DEBUG
        g = eye(4);
        init3Dplot;
    end

    % Compute tendon length
    % For each spacer disk, we compute the length of the PRECEDING segment
    for iDiskSeg = 1:(nDisks-1)
        %if MBSys.frameData.jointType(iDisk) == 2
        % Flexible joint (multiple tendon inputs)
        % l = MBSys.frameData.l(iFrm);
        for iT = 1:nTendons
            % Transformation from the contact point of the segment's
            % first disk to the contact point of the frame's second
            % disk
            g_tm_i1 = tendonDef.g_tm_Mat(:,:,1,iDiskSeg,iT);
            g_tm_i2 = tendonDef.g_tm_Mat(:,:,2,iDiskSeg,iT);
            g_tm_12 = g_tm_i1 \ g_disks_rel(:,:,iDiskSeg) * g_tm_i2;

            l_t_k(iT) = l_t_k(iT) + vecnorm( g_tm_12(1:3,4) );

            if DEBUG
                coordSysSE3(g*g_tm_i1, ...
                    "DrawLabels", false, "Scale", 0.05);
                coordSysSE3(g*g_disks_rel(:,:,iDiskSeg)*g_tm_i2, ...
                    "DrawLabels", false, "Scale", 0.05);
            end
        end
        if DEBUG
            g = g*g_disks_rel(:,:,iDiskSeg);
        end
        %end
    end


end
