classdef CtrlModeEnum < Simulink.IntEnumType
    enumeration
        LocalPilot   (1)
        RemotePilot  (2)
        Autonomous   (3)
    end
end