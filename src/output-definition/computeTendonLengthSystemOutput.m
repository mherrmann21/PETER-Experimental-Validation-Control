function yLC = computeTendonLengthSystemOutput(MBSys, cableDef, q)
    %% Compute the cable length outputs of a system in one time step
    arguments
        MBSys       (1,1) MBSystem

        cableDef    (1,1) MBSysTendonLengthOutputDefinition

        % Coordinates
        q         (:,1)
    end


    %% Compute tendon lengths

    %%% TODO: The output cable length vector currently always has
    %%% dimensions of overall nr. of inputs (possibly including joint
    %%% actuation), so it only properly works with single-link continuum
    %%% manipulators right now

    g_rel_k = MBSys.computeJointTransformations(q);

    if cableDef.isDiscrete
        yLC = computeTendonLengthsDiscrete(MBSys, cableDef, q, g_rel_k);
    else
        yLC = computeTendonLengthsContinuous(MBSys, g_rel_k);
    end
end


function l_c_k = computeTendonLengthsContinuous(MBSys, g_rel_k)
    %% Compute tendon lengths assuming continuous routing along the backbone
    arguments
        MBSys   (1,1) MBSystem
        g_rel_k (4,4,:) double
    end
    l_c_k = zeros(MBSys.nInputs, 1);

    for iFrm = 1:MBSys.nFrames
        uIndices = MBSys.frameData.getUIndices(iFrm);

        if MBSys.frameData.jointType(iFrm) == 2
            % Flexible joint (multiple cable inputs)
            % l = MBSys.frameData.l(iFrm);

            for iC = 1:length(uIndices)
                % Cable configurations at adjacent nodes
                % g_cm_i1 = SE3Matrix(eye(3), MBSys.frameData.x_cm(:,1,iFrm,iC));
                % g_cm_i2 = SE3Matrix(eye(3), MBSys.frameData.x_cm(:,2,iFrm,iC));
                g_cm_i1 = eye(4);
                g_cm_i2 = eye(4);
                g_cm_i1(1:3,4) = MBSys.frameData.x_cm(:,1,iFrm,iC);
                g_cm_i2(1:3,4) = MBSys.frameData.x_cm(:,2,iFrm,iC);

                % Discrete deformation gradient cable routing
                % Tangent vector is in elements 4:6
                xi_c = cayInvSE3( g_cm_i1 \ g_rel_k(:,:,iFrm) * g_cm_i2 );

                l_c_k(uIndices(iC)) = l_c_k(uIndices(iC)) + vecnorm( xi_c(4:6,:) );
            end
        end
    end
end

function l_c_k = computeTendonLengthsDiscrete(MBSys, cableDef, q, g_rel)
    %% Compute tendon lengths assuming routing via discrete spacer disks
    % adapted code from Leander Pfeiffer
    arguments
        MBSys   (1,1) MBSystem

        cableDef (1,1) MBSysTendonLengthOutputDefinition

        q       (:,1) double
        g_rel   (4,4,:) double
    end

    nDisks = cableDef.nDisks;
    nCables = cableDef.nCables;
    l_c_k = zeros(nCables, 1);

    % Check whether the beam discretization corresponds to spacer disk
    % distribution or not
    if 0%(nDisks-1) == MBSys.nFrames
        % Spacer disks are directly attached to the frames
        % No interpolation necessary
        g_disks_rel = g_rel;
    else
        % Spacer disks may lie between segments
        % Configurations must be interpolated

        % Positions of the beam frames (including first fixed node)
        sFrames = [0; cumsum(MBSys.frameData.l)];

        % Positions of the spacer disks (including first fixed disk)
        sDisks = cableDef.sDisks;
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

    % Compute cable length
    % For each spacer disk, we compute the length of the PRECEDING segment
    for iDiskSeg = 1:(nDisks-1)
        %if MBSys.frameData.jointType(iDisk) == 2
        % Flexible joint (multiple cable inputs)
        % l = MBSys.frameData.l(iFrm);
        for iC = 1:nCables
            % Transformation from the contact point of the segment's
            % first disk to the contact point of the frame's second
            % disk
            g_cm_i1 = cableDef.g_cm_Mat(:,:,1,iDiskSeg,iC);
            g_cm_i2 = cableDef.g_cm_Mat(:,:,2,iDiskSeg,iC);
            g_cm_12 = g_cm_i1 \ g_disks_rel(:,:,iDiskSeg) * g_cm_i2;

            l_c_k(iC) = l_c_k(iC) + vecnorm( g_cm_12(1:3,4) );
        
            if DEBUG
                coordSysSE3(g*g_cm_i1, ...
                    "DrawLabels", false, "Scale", 0.05);
                coordSysSE3(g*g_disks_rel(:,:,iDiskSeg)*g_cm_i2, ...
                    "DrawLabels", false, "Scale", 0.05);
            end
        end
        if DEBUG
            g = g*g_disks_rel(:,:,iDiskSeg);
        end
        %end
    end

    
end
