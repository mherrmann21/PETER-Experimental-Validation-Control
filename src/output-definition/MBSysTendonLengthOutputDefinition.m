classdef MBSysTendonLengthOutputDefinition
    %% System output definition for a robot with tendon-driven flexible links
    properties
        % Defines whether the cables are routed continuously along the
        % backbone or in discrete segments via spacer disks
        isDiscrete (1,1) logical = true;

        % Nr. of spacer disks (INCLUDING the possibly fixed base)
        nDisks      (1,1) double {mustBePositive} = 10;

        % Nr. of cables
        nCables     (1,1) double {mustBePositive} = 3;

        % Index of the MBSystem link from which to get the cable lengths
        linkIndex   (1,1) double = 1;

        % Vector of spacer disk arc length positions
        % dimensions (nDisks, 1)
        sDisks      (:,1) double {mustBeNonnegative}

        % Relative SE3 configurations of the cable path frames (w.r.t.
        % backbone) at the two segment nodes (i.e., nodes i-1 and i for
        % flex. joint i) (in beam reference configuration)
        % Dimensions: (4, 4, 2, nDisks-1, nCables),
        g_cm_Mat        (4,4,2,:,:) {mustBeSE3MatrixArray}

        % Same as g_cm_Mat, but additionally as SE3 array
        g_cm_SE3        (2,:,:) SE3
    end

    methods
        function obj = MBSysTendonLengthOutputDefinition(cableConfig, sDisks)
            arguments
                cableConfig (:,1) MBLinkCableActuationConfig = createArray(0,1, "MBLinkCableActuationConfig");
                sDisks      (:,1) double {mustBeNonnegative} = [];
            end

            if ~isempty(sDisks)
                obj.sDisks = sDisks;
                obj.nDisks = length(sDisks);

                obj = obj.getDiskCableMountingPoints(cableConfig);
            end
        end

        function obj = getDiskCableMountingPoints(obj, cableConfig)
            arguments
                obj         (1,1)
                cableConfig (1,1) MBLinkCableActuationConfig
            end
            obj.nCables = length(cableConfig.x_m_funs);

            g_m = cableConfig.getNodeData(obj.sDisks);
            g_cm_temp = zeros(4,4,2,obj.nDisks-1, obj.nCables);
            g_cm_temp(:,:,1,:,:) = g_m(:,:,1:end-1,:);
            g_cm_temp(:,:,2,:,:) = g_m(:,:,2:end,  :);
            obj.g_cm_Mat = g_cm_temp;

            for iDisk = 1:obj.nDisks-1
                for iC = 1:obj.nCables
                    obj.g_cm_SE3(1,iDisk,iC) ...
                        = SE3(g_cm_temp(1:3,1:3,1,iDisk,iC), g_cm_temp(1:3,4,1,iDisk,iC));
                    obj.g_cm_SE3(2,iDisk,iC) ...
                        = SE3(g_cm_temp(1:3,1:3,2,iDisk,iC), g_cm_temp(1:3,4,2,iDisk,iC));
                end
            end
        end
    end
end