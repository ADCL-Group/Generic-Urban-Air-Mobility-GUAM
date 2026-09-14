
% File description
% Name : mot_ctrl_dmd_fixedpitch_fixedwing_init			
% Author : Luke Heyerdahl - ANL		                          				
% Description : Sets PID values for the propeller motor controller

% Proprietary : public
% Protected: false

% Models :  mot_ctrl_dmd_uas -- (parent)

% Vehicle Type : rotorcraft

%%
% noEM = 6;
% power_TO = 1.0549e+05*SimIn.Units.W; % takeoff power %ft-lbf/s
% speed_base = 77.8947;  % base speed of EM rad/s
% torque_TO = power_TO/speed_base; % takeoff torque %lbf-ft
% 
% 
% % mass of the electric motor regression (NASA NDARC) pg311
% % massEM = @(Q) (0.3928)*(Q*0.7375621492772656)^(0.8587)*.454;
% massEM_fun = @(Q) 0.3928 * (Q)^(0.8587) * SimIn.Units.lbm; %slugs
% massEM = massEM_fun(torque_TO);
% % mass of the inverter regression (Duffy model, NASA NDARC) pg312
% % massInv = @(P) 0.125*(P/1000)^(0.96);
% massInv_fun = @(P) 0.125 * (P/(1000*SimIn.Units.W))^0.96 * SimIn.Units.kg;%slugs
% massInv = massInv_fun(power_TO);

%above not needed setting PID values here

powerplant.mot.ctrl.init.P = 100*0.08;
powerplant.mot.ctrl.init.I = 100*0.014;
powerplant.mot.ctrl.init.D = 100*0.002;
% powerplant.mot.plant.init.motormass = massEM + massInv;            % kg   % Mass of motor

clear power_TO speed_base torque_TO massEM massInv