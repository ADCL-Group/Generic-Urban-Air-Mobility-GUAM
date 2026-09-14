% File description
% Name : mot_ctrl_dmd_fixedpitch_fixedwing_init			
% Author : Luke Heyerdahl - ANL		                          				
% Description : Sets PID values for the propeller motor controller

% Proprietary : public
% Protected: false

% Models :  mot_ctrl_dmd_uas -- (parent)
%           powerplant_quadcopter_init

% Vehicle Type : rotorcraft

mot.ctrl.init.P = powerplant.mot.ctrl.init.P;
mot.ctrl.init.I = powerplant.mot.ctrl.init.I;
mot.ctrl.init.D = powerplant.mot.ctrl.init.D;

mot.propel.plant.init.Pmax          = powerplant.propel.plant.init.Pmax;
% mot.propel.plant.init.CP_CT_map     = powerplant.propel.plant.init.CP_CT_map;
mot.propel.init.CPpoly              = powerplant.propel.init.CPpoly;
mot.propel.plant.init.propD         = powerplant.propel.plant.init.propD;

mot.tc.plant.init.ratio = powerplant.tc.plant.init.ratio;

mot.propel.init.disableEffMap = 0; %1 sets eff to 1  |  0 sets eff to map value 