classdef MBSysTendonLengthOutputDefinition
    %% System output definition for a robot with tendon-driven flexible links
    properties
        % Defines whether the tendons are routed continuously along the
        % backbone or in discrete segments via spacer disks
        isDiscrete (1,1) logical = true;

        % Nr. of spacer disks (INCLUDING the possibly fixed base)
        nDisks      (1,1) double {mustBePositive} = 10;

        % Nr. of tendons
        nTendons    (1,1) double {mustBePositive} = 3;

        % Index of the MBSystem link from which to get the tendon lengths
        linkIndex   (1,1) double = 1;

        % Vector of spacer disk arc length positions
        % dimensions (nDisks, 1)
        sDisks      (:,1) double {mustBeNonnegative}

        % Relative SE3 configurations of the tendon path frames (w.r.t.
        % backbone) at the two segment nodes (i.e., nodes i-1 and i for
        % flex. joint i) (in beam reference configuration)
        % Dimensions: (4, 4, 2, nDisks-1, nTendons),
        g_tm_Mat        (4,4,2,:,:) {mustBeSE3MatrixArray}

        % Same as g_tm_Mat, but additionally as SE3 array
        g_tm_SE3        (2,:,:) SE3
    end

    methods
        function obj = MBSysTendonLengthOutputDefinition(tendonConfig, sDisks)
            arguments
                tendonConfig (:,1) MBLinkCableActuationConfig = createArray(0,1, "MBLinkCableActuationConfig");
                sDisks      (:,1) double {mustBeNonnegative} = [];
            end

            if ~isempty(sDisks)
                obj.sDisks = sDisks;
                obj.nDisks = length(sDisks);

                obj = obj.getDiskTendonMountingPoints(tendonConfig);
            end
        end

        function obj = getDiskTendonMountingPoints(obj, tendonConfig)
            arguments
                obj         (1,1)
                tendonConfig (1,1) MBLinkCableActuationConfig
            end
            obj.nTendons = length(tendonConfig.x_m_funs);

            g_m = tendonConfig.getNodeData(obj.sDisks);
            g_tm_temp = zeros(4,4,2,obj.nDisks-1, obj.nTendons);
            g_tm_temp(:,:,1,:,:) = g_m(:,:,1:end-1,:);
            g_tm_temp(:,:,2,:,:) = g_m(:,:,2:end,  :);
            obj.g_tm_Mat = g_tm_temp;

            for iDisk = 1:obj.nDisks-1
                for iT = 1:obj.nTendons
                    obj.g_tm_SE3(1,iDisk,iT) ...
                        = SE3(g_tm_temp(1:3,1:3,1,iDisk,iT), g_tm_temp(1:3,4,1,iDisk,iT));
                    obj.g_tm_SE3(2,iDisk,iT) ...
                        = SE3(g_tm_temp(1:3,1:3,2,iDisk,iT), g_tm_temp(1:3,4,2,iDisk,iT));
                end
            end
        end
    end
end
