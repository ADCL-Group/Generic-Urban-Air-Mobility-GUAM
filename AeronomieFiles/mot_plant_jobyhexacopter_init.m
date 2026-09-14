%% Electric motor — estimated NASA GUAM Lift+Cruise lift motor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Estimated per-motor ratings.
% Approximately 116 kW/rotor is required at nominal hover.
% 135 kW peak provides approximately 16% control/power margin.
% power_TO = 100100 ;%*5.25; %* SimIn.Units.W;       % [ft-lbf/s], peak
power_TO = 530*550;                                %530hp
power_MC = power_TO/1.25; %* SimIn.Units.W;       % [ft-lbf/s], continuous


% GUAM S-function rotor actuator permits speeds up to 350 rad/s.
% Base speed is selected near the GUAM polynomial rotor limit:
% 1600 rpm = 167.55 rad/s.
speed_base = 167.55;                       % [rad/s]
speed_max  = SimIn.Eng.PosLim_hi(9);                       % [rad/s]

torque_TO = power_TO/speed_base;        % [lbf-ft], approximately 433 lbf-ft
torque_MC = power_MC/speed_base;        % [lbf-ft], approximately 366 lbf-ft

voltage = 600;                          % [V], engineering estimate
eta_EM_max = 0.93;                      % [-], matches GUAM powertrain estimate

%% Motor and inverter mass estimates

% NASA NDARC electric-motor mass regression.
% Input Q is lbf-ft; result stored internally as slug.
massEM_fun = @(Q) ...
    0.3928 .* Q.^0.8587 .* SimIn.Units.lbm;

massEM = massEM_fun(torque_TO);

% Duffy inverter mass regression.
% Regression power input is kW; result stored internally as slug.
massInv_fun = @(P) ...
    0.125 .* (P./(1000.*SimIn.Units.W)).^0.96 .* SimIn.Units.kg;

massInv = massInv_fun(power_TO);

%% Efficiency map

% McDonald electric-propulsion loss model.
k0 = 0.5;
eta_hat = eta_EM_max;
omega_hat = speed_base;                 % [rad/s]
Q_hat = torque_TO/2;                    % [lbf-ft]

C0 = k0*omega_hat*Q_hat/6*(1-eta_hat)/eta_hat;
C1 = -3*C0/(2*omega_hat) ...
   + Q_hat*(1-eta_hat)/(4*eta_hat);

C2 = C0/(2*omega_hat^3) ...
   + Q_hat*(1-eta_hat)/(4*eta_hat*omega_hat^2);

C3 = omega_hat*(1-eta_hat)/(2*Q_hat*eta_hat);

omega_array = linspace(-speed_max,speed_max,50);
Q_array = linspace(-torque_TO,torque_TO,50);

[omega_grid,Q_grid] = meshgrid(omega_array,Q_array);

P_L = C0 ...
    + C1.*omega_grid ...
    + C2.*omega_grid.^3 ...
    + C3.*Q_grid.^2;

P_mech = omega_grid.*Q_grid;

etaEM = zeros(size(P_mech));

nonzero_power = abs(P_mech) > eps;
etaEM(nonzero_power) = ...
    abs(P_mech(nonzero_power)./ ...
    (P_mech(nonzero_power) + abs(P_L(nonzero_power))));

etaEM = min(eta_hat,max(0.01,etaEM));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Electric motor properties

mot.plant.init.mass.motor = massEM;              % [slug]
mot.plant.init.mass.controller = massInv;         % [slug]
mot.plant.init.mass.total = massEM + massInv;     % [slug]

mot.plant.init.pwr_density = ...
    power_TO/(massEM + massInv);                  % [ft-lbf/s/slug]

mot.plant.init.curr_max = ...
    (power_TO/SimIn.Units.W)/voltage;             % [A]

mot.plant.init.spd_base = speed_base;             % [rad/s]

% Motor rotor inertia only.
% Do not use the GUAM propeller inertia here unless the propeller inertia
% is absent from the mechanical load model.
mot.plant.init.inertia = ...
    0.03*SimIn.Units.kg*SimIn.Units.m2;           % [slug-ft^2], estimate

mot.plant.init.coeff_regen = 1;
mot.plant.init.volt_min = voltage;

% GUAM actuator model uses wn = 4*pi rad/s.
% 1/wn is approximately 0.08 seconds.
mot.plant.init.time_response = 1/(4*pi);          % [s]

% Prevent the unexplained value of 600 from becoming a torque cap.
mot.plant.init.t_max_trq = torque_TO;              % [lbf-ft]

%% Torque-speed curves

% Include the base-speed point and maximum-speed point only once.
speed_array = linspace(speed_base,speed_max,50);

mot.plant.init.peak_to_cont_ratio = power_TO/power_MC;

% Continuous curve:
% constant torque below base speed, constant power above base speed.
mot.plant.init.trq_cont.idx1_spd = ...
    [0 speed_array];

mot.plant.init.trq_cont.map = ...
    [torque_MC power_MC./speed_array];

% Peak curve:
% constant peak torque below base speed, constant peak power above.
mot.plant.init.trq_max.idx1_spd = ...
    mot.plant.init.trq_cont.idx1_spd;

mot.plant.init.trq_max.map = ...
    [torque_TO power_TO./speed_array];

mot.plant.init.trq_max.points.trq = torque_TO;

mot.plant.init.trq_min.idx1_spd = ...
    mot.plant.init.trq_max.idx1_spd;

mot.plant.init.trq_min.map = ...
    -mot.plant.init.trq_max.map;

%% Four-quadrant continuous torque map

mot.plant.init.trq_pos_cont.idx1_spd = ...
    [-fliplr(mot.plant.init.trq_cont.idx1_spd(2:end)), ...
     -eps,0,eps, ...
      mot.plant.init.trq_cont.idx1_spd(2:end)];

mot.plant.init.trq_pos_cont.map = ...
    [-fliplr(mot.plant.init.trq_cont.map(2:end)), ...
     -mot.plant.init.trq_cont.map(1), ...
      mot.plant.init.trq_cont.map(1), ...
      mot.plant.init.trq_cont.map(1), ...
      mot.plant.init.trq_cont.map(2:end)];

mot.plant.init.pwr_pos_cont.map = ...
    mot.plant.init.trq_pos_cont.idx1_spd .* ...
    mot.plant.init.trq_pos_cont.map;

%% Four-quadrant peak torque map

mot.plant.init.trq_pos_max.idx1_spd = ...
    [-fliplr(mot.plant.init.trq_max.idx1_spd(2:end)), ...
     -eps,0,eps, ...
      mot.plant.init.trq_max.idx1_spd(2:end)];

mot.plant.init.trq_pos_max.map = ...
    [-fliplr(mot.plant.init.trq_max.map(2:end)), ...
     -mot.plant.init.trq_max.map(1), ...
      mot.plant.init.trq_max.map(1), ...
      mot.plant.init.trq_max.map(1), ...
      mot.plant.init.trq_max.map(2:end)];

mot.plant.init.pwr_pos_max.map = ...
    mot.plant.init.trq_pos_max.idx1_spd .* ...
    mot.plant.init.trq_pos_max.map;

%% Negative continuous torque map

mot.plant.init.trq_neg_cont.idx1_spd = ...
    mot.plant.init.trq_pos_cont.idx1_spd;

mot.plant.init.trq_neg_cont.map = ...
    [fliplr(mot.plant.init.trq_cont.map(2:end)), ...
      mot.plant.init.trq_cont.map(1), ...
     -mot.plant.init.trq_cont.map(1), ...
     -mot.plant.init.trq_cont.map(1), ...
     -mot.plant.init.trq_cont.map(2:end)];

mot.plant.init.pwr_neg_cont.map = ...
    mot.plant.init.trq_neg_cont.idx1_spd .* ...
    mot.plant.init.trq_neg_cont.map;

%% Negative peak torque map

mot.plant.init.trq_neg_max.idx1_spd = ...
    mot.plant.init.trq_pos_max.idx1_spd;

mot.plant.init.trq_neg_max.map = ...
    [fliplr(mot.plant.init.trq_max.map(2:end)), ...
      mot.plant.init.trq_max.map(1), ...
     -mot.plant.init.trq_max.map(1), ...
     -mot.plant.init.trq_max.map(1), ...
     -mot.plant.init.trq_max.map(2:end)];

mot.plant.init.pwr_neg_max.map = ...
    mot.plant.init.trq_neg_max.idx1_spd .* ...
    mot.plant.init.trq_neg_max.map;

%% Efficiency and electrical-power maps

mot.plant.init.eff_trq.idx1_spd = omega_array;
mot.plant.init.eff_trq.idx2_trq = Q_array;
mot.plant.init.eff_trq.map = etaEM;

mot.plant.calc.eff_trq.max = eta_hat;

mot.plant.calc.pwr_elec.idx1_spd = omega_array;
mot.plant.calc.pwr_elec.idx2_trq = Q_array;

mot.plant.calc.pwr_elec.map = zeros(size(P_mech));

motoring = P_mech > 0;
generating = P_mech < 0;

mot.plant.calc.pwr_elec.map(motoring) = ...
    P_mech(motoring)./etaEM(motoring);

mot.plant.calc.pwr_elec.map(generating) = ...
    P_mech(generating).*etaEM(generating);

%% Clear temporary variables

clear power_TO power_MC speed_base speed_max
clear torque_TO torque_MC voltage eta_EM_max
clear massEM massInv massEM_fun massInv_fun
clear k0 eta_hat omega_hat Q_hat
clear C0 C1 C2 C3
clear omega_array Q_array omega_grid Q_grid
clear P_L P_mech etaEM nonzero_power
clear speed_array motoring generating