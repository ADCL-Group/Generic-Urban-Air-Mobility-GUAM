function [vehicle, CtrlTrim] = build_vehicle(SimIn)
    if ~isfield(SimIn,'ScalingFactor') || isempty(SimIn.ScalingFactor)
        rawScale = 1;
    else
        rawScale = SimIn.ScalingFactor;
    end
    
    if ~isfield(SimIn,'vehID') || isempty(SimIn.vehID)
        vehID = -1;
    else
        vehID = SimIn.vehID;
    end
    
    if rawScale > 0
        buildScale = rawScale;
    else
        buildScale = 1;
    end
    
    switch vehID
        case 1
            vehicle = build_Hero();
            CtrlTrim.trim_vel = 0:2:50;
            CtrlTrim.V_TRC  = 5;  
            CtrlTrim.V_hover_exit  = 10;        
            CtrlTrim.V_hover_enter = 15;
            CtrlTrim.V_cruise_exit = 30;
            CtrlTrim.V_cruise_enter= 36;
        otherwise
            vehicle = build_Lift_plus_Cruise(buildScale);
            speedScale = sqrt(buildScale);
            CtrlTrim.trim_vel = [0:5:90,94.8,95,100:5:130] * speedScale; 
            CtrlTrim.V_TRC  = 20 * speedScale;         
            CtrlTrim.V_hover_exit  = 30 * speedScale;        
            CtrlTrim.V_hover_enter = 35 * speedScale;
            CtrlTrim.V_cruise_exit = 85 * speedScale;
            CtrlTrim.V_cruise_enter= 90 * speedScale;
    end
end