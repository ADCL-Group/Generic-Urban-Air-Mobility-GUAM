% %% File description
% % Name : mot_plant_LC_Lift_init (CORRECTED)
% % Description : Motor parameters scaled for NASA Lift+Cruise (Lift Rotor)
% % Target: ~90kW Peak / 70kW Cont, ~1050 RPM Hover, ~820 Nm Peak Torque
% 
% %% File content 
% 
% % --- 1. PHYSICAL PROPERTIES ---
% % Inertia: Matches NASA Ref (0.0208 slug-ft^2)
% mot.plant.init.inertia = 0.028;         % kg*m^2 
% % Mass: Scaled for 90kW peak power (assuming ~4.5 kW/kg technology)
% mot.plant.init.mass.motor = 20.0;       % kg 
% mot.plant.init.mass.controller = 5.0;   % kg
% mot.plant.init.mass.total = 25.0;       % kg 
% 
% % --- 2. ELECTRICAL & THERMAL LIMITS ---
% mot.plant.init.volt_min = 500;          % Increased voltage for higher power
% mot.plant.init.pwr_density = 3000;      % W/kg (Aggressive aviation grade)
% mot.plant.init.curr_max = 250;          % Amps
% 
% % --- 3. SPEED & TORQUE SCALING ---
% % Hover is ~110 rad/s. Max speed set to ~125 rad/s (1200 RPM) for control margin.
% mot.plant.init.spd_base = 125.0;        % rad/s 
% 
% % --- 4. TORQUE MAPS ---
% % Speed vector: 0 to 160 rad/s (approx 1500 RPM)
% spd_vector = linspace(0, 160, 20);      
% 
% mot.plant.init.trq_cont.idx1_spd = spd_vector;
% mot.plant.init.trq_max.idx1_spd = spd_vector;
% mot.plant.init.trq_min.idx1_spd = spd_vector;
% 
% % Peak Torque: 820 Nm (Allows maneuvering above the 630 Nm hover load)
% % Continuous Torque: 650 Nm (Slightly above the 628 Nm hover load)
% mot.plant.init.trq_max.map = [820 820 820 820 820 820 820 820 820 810 ...
%                               800 780 750 700 600 500 400 300 0 0];
% 
% mot.plant.init.trq_cont.map = [650 650 650 650 650 650 650 650 650 650 ...
%                                640 630 620 580 500 400 300 200 0 0];
% 
% mot.plant.init.trq_min.map = -1 * mot.plant.init.trq_max.map; 
% 
% mot.plant.init.trq_max.points.trq = 20;
% 
% % --- 5. EFFICIENCY MAP ---
% % Flat 94% efficiency (Typical for high-end brushless aviation motors)
% mot.plant.init.eff_trq.idx1_spd = [-1*flip(spd_vector(2:end)) spd_vector]; 
% mot.plant.init.eff_trq.idx2_trq = linspace(-820, 820, 17);
% 
% mot.plant.init.eff_trq.map = 0.94 * ones(length(mot.plant.init.eff_trq.idx2_trq), ...
%                                          length(mot.plant.init.eff_trq.idx1_spd));
% 
% % --- 6. POWER LIMITS ---
% % Peak Power ~ 90 kW (Required for maneuvering)
% % Cont Power ~ 70 kW (Required for hover)
% mot.plant.init.pwr_pos_max = struct;
% mot.plant.init.pwr_pos_max.map = 90000 * ones(size(spd_vector)); 
% mot.plant.init.pwr_pos_cont = struct;
% mot.plant.init.pwr_pos_cont.map = 70000 * ones(size(spd_vector));
% 
% % Regen Limits
% mot.plant.init.pwr_neg_max = struct;
% mot.plant.init.pwr_neg_max.map = -90000 * ones(size(spd_vector));
% mot.plant.init.pwr_neg_cont = struct;
% mot.plant.init.pwr_neg_cont.map = -70000 * ones(size(spd_vector));
% 
% % Copy limits for negative direction
% mot.plant.init.trq_neg_cont = mot.plant.init.trq_cont; 
% mot.plant.init.trq_neg_cont.idx1_spd = -1 * spd_vector;
% mot.plant.init.trq_neg_cont.map = -1 * mot.plant.init.trq_cont.map;
% 
% mot.plant.init.trq_neg_max = mot.plant.init.trq_max;
% mot.plant.init.trq_neg_max.idx1_spd = -1 * spd_vector;
% mot.plant.init.trq_neg_max.map = -1 * mot.plant.init.trq_max.map;

%=============================================================================================================

% File description
% Name : quadcopter_init (ADAPTED for NASA L+C Lift Rotor)
% Description : Initialize parameters for ONE Lift Rotor of the NASA L+C
% Note: This file replaces the pointers with hardcoded NASA values.

%% Geometry & Mass
powerplant.propel.plant.init.rotorcorediam = 0.30*SimIn.Units.m;      % ft     % Hub diameter (approx 30cm)
powerplant.propel.plant.init.Wprop = 2.0*SimIn.Units.kg*SimIn.Environment.Earth.Gravity.g0;              % lbf     % Propeller weight (approx 2kg)
powerplant.propel.plant.init.Iprop = 0.025*SimIn.Units.kg*SimIn.Units.m2;             % slug-ft^2 % Propeller Inertia (Bulk of the 0.028 total)
powerplant.propel.plant.init.propD = SimIn.Model.Prop{9, 1}.Dp;              % ft     % Diameter (9 ft)

%% Rotation Direction (CRITICAL: NASA L+C alternates)
% You must manually assign this for each of the 8 rotors. 
% Standard L+C Pattern: Front-Right CCW, Front-Left CW, etc.
% Setting generic defaults here, but OVERWRITE these in your vehicle setup.
% powerplant.propel.plant.init.rotationdir1 = [0; 0; 1];  % CCW
% powerplant.propel.plant.init.rotationdir2 = [0; 0; -1]; % CW
% powerplant.propel.plant.init.rotationdir3 = [0; 0; 1];
% powerplant.propel.plant.init.rotationdir4 = [0; 0; -1]; % (Add 5-8 if your struct supports it)

% powerplant.propel.plant.init.thrustdir = [0; 0; -1];    % Thrust direction (Upwards force, -Z in NED)

powerplant.propel.plant.init.thrustdir = SimIn.Model.Prop{9, 1}.e_b;    % Thrust direction (Upwards force, -Z in NED)

%% Performance Limits
% powerplant.propel.plant.init.Pmax = 100100 ;%* 5.25;              % ft-lbf/s     % Max Power (90 kW)(250kw)
powerplant.propel.plant.init.Pmax = 530*550; %530hp

%% Aerodynamic Maps (CP / CT)
load('vehicles/Lift+Cruise/AeroProp/SFunction/PropCoef/APCSF_10x4p7_coef.mat')
powerplant.propel.init.CTpoly = APCSF_10x4p7_coef(:,1);
powerplant.propel.init.CPpoly = APCSF_10x4p7_coef(:,2);
powerplant.propel.init.effpoly = [0 0.75];

%% Controller Initial Conditions (Trim State)
% These set the starting point for the simulation so it doesn't crash on start.
% Calculated for NASA L+C Hover:
powerplant.propel.ctrl.init.rps = 17.5;         % rev/s % (1050 RPM)
powerplant.propel.ctrl.init.trq_trim = 628.0*SimIn.Units.N*SimIn.Units.m;   % N-m   % Torque required to hover
powerplant.propel.ctrl.init.hover_dt = [0.7 0.7]; % Estimated Duty Cycle (0.0-1.0) for hover

clear APCSF_10x4p7_coef
