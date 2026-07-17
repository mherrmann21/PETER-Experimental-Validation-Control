classdef MBControlSystem
    %% Multibody Control System Class
    % to represent a "control system" comprised of a multibody system and
    % output definitions
    properties
        % System definition
        % Can be both numeric or symbolic MBSystem
        MBSys       (1,1) MBSystem = MBSystemNum;

        %% Output definitions
        IMUDef      (1,1) MBSysIMUOutputDefinition

        tendonDef   (1,1) MBSysTendonLengthOutputDefinition
    end

    methods
        function obj = MBControlSystem(MBSys, IMUDef, tendonDef)
            arguments
                MBSys       (1,1) MBSystem = MBSystemNum;
                IMUDef      (1,1) MBSysIMUOutputDefinition = MBSysIMUOutputDefinition;
                tendonDef   (1,1) MBSysTendonLengthOutputDefinition = MBSysTendonLengthOutputDefinition;
            end
            obj.MBSys = MBSys;
            obj.IMUDef = IMUDef;
            obj.tendonDef = tendonDef;
        end
    end
end