function yLt = computeTendonLengthSystemOutput_casadi(MBSys, tendonDef,...
        q, g_rel)
    %% Compute the outputs of a system in one time step
    arguments
        MBSys       (1,1) MBSystemSym

        tendonDef   (1,1) MBSysTendonLengthOutputDefinition

        % Coordinates and joint transformations
        q         (:,1)
        g_rel     (:,1) SE3
    end

    %% Compute tendon lengths

    %%% TODO: The output tendon length vector currently always has
    %%% dimensions of overall nr. of inputs (possibly including joint
    %%% actuation), so it only properly works with single-link continuum
    %%% manipulators right now

    if tendonDef.isDiscrete
        yLt = computeTendonLengthsDiscrete(MBSys, tendonDef, q, g_rel);
    else
        yLt = computeTendonLengthsContinuous(MBSys, g_rel);
    end

end


function l_t_k = computeTendonLengthsContinuous(MBSys, g_rel)
    %% Compute tendon lengths assuming continuous routing along the backbone
    arguments
        MBSys   (1,1) MBSystem
        g_rel   (:,1) SE3
    end

    f = getSE3Functions(g_rel.x);

    l_t_k = zeros(MBSys.nInputs, 1);

    for iFrm = 1:MBSys.nFrames
        uIndices = MBSys.frameData.getUIndices(iFrm);

        if MBSys.frameData.jointType(iFrm) == 2
            % Flexible joint (multiple tendon inputs)
            % l = MBSys.frameData.l(iFrm);

            for iT = 1:length(uIndices)
                % Tendon configurations at adjacent nodes
                g_tm_i1 = SE3(f.eye(3), MBSys.frameData.x_cm(:,1,iFrm,iT));
                g_tm_i2 = SE3(f.eye(3), MBSys.frameData.x_cm(:,2,iFrm,iT));

                % Discrete deformation gradient tendon routing
                g_rel_t = g_tm_i1 \ g_rel(iFrm) * g_tm_i2;
                [~, v_t] = f.cayInvSE3( g_rel_t.R, g_rel_t.x );

                l_t_k(uIndices(iT)) = l_t_k(uIndices(iT)) + vecnorm( v_t );
            end
        end
    end
end

function Lt = computeTendonLengthsDiscrete(MBSys, tendonDef, q, g_rel)
    %% Compute tendon lengths assuming routing via discrete spacer disks
    % adapted code from Leander Pfeiffer
    arguments
        MBSys    (1,1) MBSystem
        tendonDef (1,1) MBSysTendonLengthOutputDefinition
        q        (:,1)
        g_rel    (:,1) SE3
    end
    f = getSE3Functions(g_rel(1).x);
    f.eye   = @casadi.MX.eye;
    f.zeros = @casadi.MX.zeros;

    nDisks   = tendonDef.nDisks;
    nTendons = tendonDef.nTendons;
    Lt = f.zeros(nTendons, 1);

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

        g_disks_rel = createArray(nDisks, 1, "SE3");
        for iDiskSeg = 1:(nDisks-1)
            % Get frame before IMU
            iFrmSD = find(sDisks(iDiskSeg+1) > sFrames, 1, "last");

            % Get discrete deformation
            qIndices = double(MBSys.frameData.qIndices(1,iFrmSD):MBSys.frameData.qIndices(2,iFrmSD));
            Ba = MBSys.frameData.Ba(iFrmSD);
            om  = (Ba(1:3,:) * q(qIndices) + MBSys.frameData.xiC(1:3,iFrmSD));
            v   = (Ba(4:6,:) * q(qIndices) + MBSys.frameData.xiC(4:6,iFrmSD));

            % Transformation w.r.t. previous spacer disk
            [g_disks_rel(iDiskSeg).R, g_disks_rel(iDiskSeg).x] = ...
                f.caySE3(om*lDisks(iDiskSeg), v*lDisks(iDiskSeg));
        end
    end

    % Compute tendon length
    % For each spacer disk, we compute the length of the PRECEDING segment
    for iDiskSeg = 1:(nDisks-1)
        %if MBSys.frameData.jointType(iFrm) == 2
        % Flexible joint (multiple tendon inputs)
        % l = MBSys.frameData.l(iFrm);
        for iT = 1:nTendons
            % Transformation from the contact point of the segment's
            % first disk to the contact point of the frame's second
            % disk
            g_tm_i1 = tendonDef.g_tm_SE3(1,iDiskSeg,iT);
            g_tm_i2 = tendonDef.g_tm_SE3(2,iDiskSeg,iT);

            g_tm_12 = g_tm_i1 \ g_disks_rel(iDiskSeg) * g_tm_i2;

            Lt(iT) = Lt(iT) + norm( g_tm_12.x );
        end
        %end
    end
end
