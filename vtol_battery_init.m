powerplant.accelec.plant.init.pwr = 40e3; % (Watts)
% powerplant.accelec.plant.init.mass.ess_12v = 10;
% powerplant.accelec.plant.init.mass.acc = 10;
% powerplant.accelec.plant.init.mass.total = 10;

accelec.plant.init.pwr = powerplant.accelec.plant.init.pwr;
% accelec.plant.init.mass.ess_12v = powerplant.accelec.plant.init.mass.ess_12v;
% accelec.plant.init.mass.acc = powerplant.accelec.plant.init.mass.acc;
% accelec.plant.init.mass.total = powerplant.accelec.plant.init.mass.total;

pxf_diagram_options.workspace_sample_time = SimIn.Environment.Turbulence.dT;
pxf_diagram_options.decimation = 10;

%%%%% Battery sizing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% References:
% [1]: https://rotorcraft.arc.nasa.gov/Publications/files/vtol-urban-air-2.pdf
% [2]: https://doi.org/10.2514/1.C037044
% [3]: https://ntrs.nasa.gov/api/citations/20205004194/downloads/VFS-F76(2020)_Snyder-68_final.pdf
lb2kg = 0.454;

Vbus = 600;

% mass_battery = 1693*lb2kg; %vtol value [1]
% power_battery = 829e3; % battery power required in hover
E_total_J = 1436e6; % battery capacity (J) VTOL value [1]
no_batt = 1; % number of batteries [2] should be five but the sim block represents a single one so might as well use only one block rather than multiplying everything by 5
no_cell_series = ceil(Vbus/3.6); % minimum number of cells in series

E_pack_J = E_total_J / no_batt; % Energy per battery

C_cell = 2;  % your chosen cell Ah
E_cell_J = 3.6*C_cell*3600; % energy per cell
no_cells = ceil(E_pack_J / E_cell_J); % parallel strings per pack
no_cells_parallel = ceil(no_cells/no_cell_series);
                                 
ess.plant.init.chemistry = 'NMC622-G';
ess.plant.init.design_num_module_parallel = no_cells_parallel;
ess.plant.init.technology = 'liion';
ess.plant.init.soc_init = 0.99;
ess.plant.init.element_per_module = 1; % do not know if it is actually used
ess.plant.init.num_module = 1; % do not know if it is actually used
ess.plant.init.design_num_cell_series = no_cell_series;
ess.plant.init.soc_min = 0.14;
ess.plant.init.soc_max = 0.99;
ess.plant.init.volt_nom = 3.6;
ess.plant.init.cap = C_cell;
ess.plant.init.packaging_factor = 1.1;
ess.plant.init.battery_oversizing_factor_erg = 1;
ess.plant.init.battery_oversizing_factor_pwr = 1;
ess.plant.init.soc_window = ess.plant.init.soc_max - ess.plant.init.soc_min;

ess.plant.init.soc_index = [0:0.1:1]; % SOC RANGE over which data is defined

ess.plant.init.voc.idx1_soc = ess.plant.init.soc_index;
ess.plant.init.voc.map = [3 3.3683 3.4492 3.5149 3.5671 3.6168 3.6787 3.7576 ...
                         3.8359 3.9058 3.9955];

ess.plant.init.rint.idx1_soc = ess.plant.init.soc_index;
ess.plant.init.rint.map = [0.0024728 0.0021687 0.0017713000000000002 0.0016368 ...
                          0.001595 0.0015857000000000002 0.0016135 0.0016727 ...
                          0.0017184 0.0017399 0.0017034];

ess.plant.init.rpol1.idx1_soc = ess.plant.init.soc_index;
ess.plant.init.rpol1.map = [0.0048684 0.0022397 0.000787 0.0006333 0.000631 ...
                           0.0006389 0.0006551 0.0007163 0.0008162 0.000783 ...
                           0.0005934];

ess.plant.init.rpol2.idx1_soc = ess.plant.init.soc_index;
ess.plant.init.rpol2.map = [0.0089754 0.0013962 0.0017452 0.0017452 ...
                           0.0013962 0.0012466 0.0012964 0.0014959 ...
                           0.0012964 0.0010471 0.0008975];

ess.plant.init.tau1.idx1_soc = ess.plant.init.soc_index;
ess.plant.init.tau1.map = [22.8 22.8 22.8 22.8 22.8 22.8 22.8 22.8 22.8 ...
                          22.8 22.8];

ess.plant.init.tau2.idx1_soc = ess.plant.init.soc_index;
ess.plant.init.tau2.map = [270 270 270 270 270 270 270 270 270 270 270];

% Max and min,max and min voltage when charging/discharging
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ess.plant.init.curr_dis_cont_max.map   = 20; %Maximal continuous discharge current (A)
ess.plant.init.curr_dis_burst_max.map  = 40; %Maximal burst discharge current(A)

ess.plant.init.volt_min_cont       = 2.7; %minimal continuous discharge voltage(V)
ess.plant.init.volt_min_burst      = 2.3; %minimal burst discharge voltage(V)

ess.plant.init.volt_max_cont       = 4.0; %maximal continuous charge voltage(V)
ess.plant.init.volt_max_burst      = 4.1; %maximal burst charge voltage(V)

ess.plant.init.curr_chg_cont_max.map   = -20; %Maximal continuous charge current (NEGATIVE)(A)
ess.plant.init.curr_chg_burst_max.map  = -40; %Maximal burst charge current (NEGATIVE)(A)

%Constraint PI parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ess.plant.init.time_burst_max      = 10; % (s) maximal burst time (= the time it takes for the constraint to change from 400 to 150 A, when a constant current of 400A is applied)
ess.plant.init.time_cont_min       = 30; % (s) time during which the constraint is blocked to continuous limit after a burst
%respected when the current passes from a high value to 0 AND when the burst time has been long enough for the PI integral reach its lower limit
%for exemple, if the maximal currents are set to 150A (cont) and 400A (burst),time_burst_max=10s, time_cont_min=30s, the minimal burst time it takes to the integral to saturate
%when a 400A burst is applied is 28s

% Max Power continous and peak when charging/discharging and update current
% max (continous and burst when charging/discharging)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear tmp1
tmp1.pwr_dis.cur_limit       = ess.plant.init.curr_dis_cont_max.map.*(ess.plant.init.voc.map - ess.plant.init.curr_dis_cont_max.map.*(ess.plant.init.rint.map + ess.plant.init.rpol1.map + ess.plant.init.rpol2.map));
tmp1.pwr_dis.vol_limit       = (ess.plant.init.voc.map - ess.plant.init.volt_min_cont ) * ess.plant.init.volt_min_cont ./(ess.plant.init.rint.map + ess.plant.init.rpol1.map + ess.plant.init.rpol2.map);
tmp1.pwr_dis.idx             = (ess.plant.init.voc.map - ess.plant.init.curr_dis_cont_max.map.*(ess.plant.init.rint.map + ess.plant.init.rpol1.map + ess.plant.init.rpol2.map)) < ess.plant.init.volt_min_cont;
ess.plant.init.pwr_dis.idx1_soc = ess.plant.init.soc_index;
ess.plant.init.pwr_dis.map             = tmp1.pwr_dis.cur_limit;
ess.plant.init.pwr_dis.map(tmp1.pwr_dis.idx) = tmp1.pwr_dis.vol_limit(tmp1.pwr_dis.idx);
ess.plant.init.pwr_dis_cont_100soc  = interp1(ess.plant.init.soc_index, ess.plant.init.pwr_dis.map, 1);
ess.plant.init.pwr_dis_max            = max((ess.plant.init.voc.map-ess.plant.init.volt_min_cont).*ess.plant.init.volt_min_cont./(ess.plant.init.rint.map+ess.plant.init.rpol1.map + ess.plant.init.rpol2.map));%per cell
ess.plant.init.curr_dis_cont_max.idx1_soc = ess.plant.init.soc_index; 
ess.plant.init.curr_dis_cont_max.map  = ess.plant.init.curr_dis_cont_max.map .* ones(size(ess.plant.init.soc_index));
ess.plant.init.curr_dis_cont_max.map(tmp1.pwr_dis.idx) = ess.plant.init.pwr_dis.map(tmp1.pwr_dis.idx) ./ ess.plant.init.volt_min_cont;
clear tmp1

tmp1.pwr_chg.cur_limit       = ess.plant.init.curr_chg_cont_max.map.*(ess.plant.init.voc.map - ess.plant.init.curr_chg_cont_max.map.*(ess.plant.init.rint.map + ess.plant.init.rpol1.map + ess.plant.init.rpol2.map));
tmp1.pwr_chg.vol_limit       = (ess.plant.init.voc.map - ess.plant.init.volt_max_cont ) * ess.plant.init.volt_max_cont ./(ess.plant.init.rint.map + ess.plant.init.rpol1.map + ess.plant.init.rpol2.map);
tmp1.pwr_chg.idx             = (ess.plant.init.voc.map - ess.plant.init.curr_chg_cont_max.map.*(ess.plant.init.rint.map + ess.plant.init.rpol1.map + ess.plant.init.rpol2.map)) > ess.plant.init.volt_max_cont;
ess.plant.init.pwr_chg.idx1_soc = ess.plant.init.soc_index;
ess.plant.init.pwr_chg.map             = tmp1.pwr_chg.cur_limit;
ess.plant.init.pwr_chg.map(tmp1.pwr_chg.idx) = tmp1.pwr_chg.vol_limit(tmp1.pwr_chg.idx);
ess.plant.init.pwr_chg_cont_30soc  = interp1(ess.plant.init.soc_index, ess.plant.init.pwr_chg.map, 0.3);
ess.plant.init.pwr_chg_max            = -max((ess.plant.init.volt_max_cont-ess.plant.init.voc.map).*ess.plant.init.volt_max_cont./(ess.plant.init.rint.map+ess.plant.init.rpol1.map + ess.plant.init.rpol2.map));%per cell
ess.plant.init.curr_chg_cont_max.idx1_soc = ess.plant.init.soc_index; 
ess.plant.init.curr_chg_cont_max.map  = ess.plant.init.curr_chg_cont_max.map .* ones(size(ess.plant.init.soc_index));
ess.plant.init.curr_chg_cont_max.map(tmp1.pwr_chg.idx) = ess.plant.init.pwr_chg.map(tmp1.pwr_chg.idx) ./ ess.plant.init.volt_max_cont;
clear tmp1

ess.plant.init.curr_ratio_load2pol1 = 0.3551;
ess.plant.init.curr_ratio_load2pol2 = 0.0364;
tmp1.pwr_dis.cur_limit       = ess.plant.init.curr_dis_burst_max.map.*(ess.plant.init.voc.map - ess.plant.init.curr_dis_burst_max.map.*(ess.plant.init.rint.map + ess.plant.init.rpol1.map*ess.plant.init.curr_ratio_load2pol1 + ess.plant.init.rpol2.map*ess.plant.init.curr_ratio_load2pol2));
tmp1.pwr_dis.vol_limit       = (ess.plant.init.voc.map - ess.plant.init.volt_min_burst ) * ess.plant.init.volt_min_burst ./(ess.plant.init.rint.map + ess.plant.init.rpol1.map*ess.plant.init.curr_ratio_load2pol1 + ess.plant.init.rpol2.map*ess.plant.init.curr_ratio_load2pol2);
tmp1.pwr_dis.idx             = (ess.plant.init.voc.map - ess.plant.init.curr_dis_burst_max.map.*(ess.plant.init.rint.map + ess.plant.init.rpol1.map*ess.plant.init.curr_ratio_load2pol1 + ess.plant.init.rpol2.map*ess.plant.init.curr_ratio_load2pol2)) < ess.plant.init.volt_min_burst;
ess.plant.init.pwr_dis_10sec.idx1_soc = ess.plant.init.soc_index;
ess.plant.init.pwr_dis_10sec.map       = tmp1.pwr_dis.cur_limit;
ess.plant.init.pwr_dis_10sec.map(tmp1.pwr_dis.idx) = tmp1.pwr_dis.vol_limit(tmp1.pwr_dis.idx);
ess.plant.init.pwr_dis_10sec_100soc= interp1(ess.plant.init.soc_index,ess.plant.init.pwr_dis_10sec.map,1);
ess.plant.init.pwr_dis_10sec_20soc = interp1(ess.plant.init.soc_index,ess.plant.init.pwr_dis_10sec.map,0.2);
ess.plant.init.curr_dis_burst_max.idx1_soc = ess.plant.init.soc_index;
ess.plant.init.curr_dis_burst_max.map  = ess.plant.init.curr_dis_burst_max.map .* ones(size(ess.plant.init.soc_index));
ess.plant.init.curr_dis_burst_max.map(tmp1.pwr_dis.idx) = ess.plant.init.pwr_dis_10sec.map(tmp1.pwr_dis.idx) ./ ess.plant.init.volt_min_burst;
clear tmp1

tmp1.pwr_chg.cur_limit       = ess.plant.init.curr_chg_burst_max.map.*(ess.plant.init.voc.map - ess.plant.init.curr_chg_burst_max.map.*(ess.plant.init.rint.map + ess.plant.init.rpol1.map*ess.plant.init.curr_ratio_load2pol1 + ess.plant.init.rpol2.map*ess.plant.init.curr_ratio_load2pol2));
tmp1.pwr_chg.vol_limit       = (ess.plant.init.voc.map - ess.plant.init.volt_max_burst ) * ess.plant.init.volt_max_burst ./(ess.plant.init.rint.map + ess.plant.init.rpol1.map*ess.plant.init.curr_ratio_load2pol1 + ess.plant.init.rpol2.map*ess.plant.init.curr_ratio_load2pol2);
tmp1.pwr_chg.idx             = (ess.plant.init.voc.map - ess.plant.init.curr_chg_burst_max.map.*(ess.plant.init.rint.map + ess.plant.init.rpol1.map*ess.plant.init.curr_ratio_load2pol1 + ess.plant.init.rpol2.map*ess.plant.init.curr_ratio_load2pol2)) > ess.plant.init.volt_max_burst;
ess.plant.init.pwr_chg_10sec.idx1_soc = ess.plant.init.soc_index;
ess.plant.init.pwr_chg_10sec.map       = tmp1.pwr_chg.cur_limit;
ess.plant.init.pwr_chg_10sec.map(tmp1.pwr_chg.idx) = tmp1.pwr_chg.vol_limit(tmp1.pwr_chg.idx);
ess.plant.init.pwr_chg_10sec_30soc = interp1(ess.plant.init.soc_index,ess.plant.init.pwr_chg_10sec.map,0.3);
ess.plant.init.curr_chg_burst_max.idx1_soc = ess.plant.init.soc_index;
ess.plant.init.curr_chg_burst_max.map  = ess.plant.init.curr_chg_burst_max.map .* ones(size(ess.plant.init.soc_index));
ess.plant.init.curr_chg_burst_max.map(tmp1.pwr_chg.idx) = ess.plant.init.pwr_chg_10sec.map(tmp1.pwr_chg.idx) ./ ess.plant.init.volt_max_burst;
clear tmp1

%min current when charging
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%[DISCHARGE]  CURRENT CONSTRAIN PI CONTROLLER
ess.plant.init.ul_curr_dis         =1 ; %upper limit of the integral of the PI controller for current constrain
ess.plant.init.Ki_curr_dis         = ess.plant.init.ul_curr_dis/((ess.plant.init.curr_dis_burst_max.map-ess.plant.init.curr_dis_cont_max.map)/ess.plant.init.curr_dis_cont_max.map*ess.plant.init.time_burst_max); %integral gain (current)
ess.plant.init.ll_curr_dis         =-ess.plant.init.time_cont_min*ess.plant.init.Ki_curr_dis; %lower limit of the integral of the PI controller for current constrain
%[DISCHARGE] VOLTAGE CONSTRAIN PI CONTROLLER
ess.plant.init.ul_volt_dis         =1 ; %upper limit of the integral of the PI controller for voltage constrain
ess.plant.init.Ki_volt_dis         = ess.plant.init.ul_volt_dis/((ess.plant.init.volt_min_cont-ess.plant.init.volt_min_burst)/ess.plant.init.volt_min_cont*ess.plant.init.time_burst_max); %integral gain (voltage)
ess.plant.init.ll_volt_dis         =-ess.plant.init.time_cont_min*ess.plant.init.Ki_volt_dis*(ess.plant.init.volt_nom-ess.plant.init.volt_min_cont)/ess.plant.init.volt_min_cont; %lower limit of the integral of the PI controller for voltage constrain
                       %%
%[CHARGE]  CURRENT CONSTRAIN PI CONTROLLER
ess.plant.init.ul_curr_chg         =1 ; %upper limit of the integral of the PI controller for current constrain
ess.plant.init.Ki_curr_chg         =ess.plant.init.ul_curr_chg/((ess.plant.init.curr_chg_burst_max.map-ess.plant.init.curr_chg_cont_max.map)/ess.plant.init.curr_chg_cont_max.map*ess.plant.init.time_burst_max); %integral gain (current)
ess.plant.init.ll_curr_chg         =-ess.plant.init.time_cont_min*ess.plant.init.Ki_curr_chg; %lower limit of the integral of the PI controller for current constrain
%[CHARGE] VOLTAGE CONSTRAIN PI CONTROLLER
ess.plant.init.ul_volt_chg         =1 ; %upper limit of the integral of the PI controller for voltage constrain
ess.plant.init.Ki_volt_chg         = ess.plant.init.ul_volt_chg/((ess.plant.init.volt_max_burst-ess.plant.init.volt_max_cont)/ess.plant.init.volt_max_cont*ess.plant.init.time_burst_max); %integral gain (voltage)
ess.plant.init.ll_volt_chg         =-ess.plant.init.time_cont_min*ess.plant.init.Ki_volt_chg*(-ess.plant.init.volt_nom+ess.plant.init.volt_max_cont)/ess.plant.init.volt_max_cont; %lower limit of the integral of the PI controller for voltage constrain
% gain factor to modify ess.plant.init.pwr_chg and ess.plant.init.pwr_dis and and
% ess.plant.init.pwr_chg10sec and ess.plant.init.pwr_dis10sec
% discharge is brought to 0 at low SOC and charge is brought to 0 at high
% SOC
% modification by vfreyermuth on 9/8/06
% ess.plant.init.pwr_dis.map            = ess.plant.init.pwr_dis.map .* double(ess.plant.init.soc_index >= ess.plant.init.soc_min);
% ess.plant.init.curr_dis_cont_max.map  = ess.plant.init.curr_dis_cont_max.map .* double(ess.plant.init.soc_index >= ess.plant.init.soc_min);
% ess.plant.init.pwr_dis_10sec.map      = ess.plant.init.pwr_dis_10sec.map .* double(ess.plant.init.soc_index >= ess.plant.init.soc_min);
% ess.plant.init.curr_dis_burst_max.map = ess.plant.init.curr_dis_burst_max.map .* double(ess.plant.init.soc_index >= ess.plant.init.soc_min);
% ess.plant.init.pwr_chg.map            = ess.plant.init.pwr_chg.map .*double(ess.plant.init.soc_index <= ess.plant.init.soc_max);
% ess.plant.init.curr_chg_cont_max.map  = ess.plant.init.curr_chg_cont_max.map .*double(ess.plant.init.soc_index <= ess.plant.init.soc_max);
% ess.plant.init.pwr_chg_10sec.map      = ess.plant.init.pwr_chg_10sec.map .*double(ess.plant.init.soc_index <= ess.plant.init.soc_max);
% ess.plant.init.curr_chg_burst_max.map = ess.plant.init.curr_chg_burst_max.map .*double(ess.plant.init.soc_index <= ess.plant.init.soc_max);
ess.plant.init.pwr_dis.map            = ess.plant.init.pwr_dis.map ;%.* double(ess.plant.init.soc_index >= ess.plant.init.soc_min);
ess.plant.init.curr_dis_cont_max.map  = ess.plant.init.curr_dis_cont_max.map ;%.* double(ess.plant.init.soc_index >= ess.plant.init.soc_min);
ess.plant.init.pwr_dis_10sec.map      = ess.plant.init.pwr_dis_10sec.map ;%.* double(ess.plant.init.soc_index >= ess.plant.init.soc_min);
ess.plant.init.curr_dis_burst_max.map = ess.plant.init.curr_dis_burst_max.map;% .* double(ess.plant.init.soc_index >= ess.plant.init.soc_min);
ess.plant.init.pwr_chg.map            = ess.plant.init.pwr_chg.map;%.*double(ess.plant.init.soc_index <= ess.plant.init.soc_max);
ess.plant.init.curr_chg_cont_max.map  = ess.plant.init.curr_chg_cont_max.map;% .*double(ess.plant.init.soc_index <= ess.plant.init.soc_max);
ess.plant.init.pwr_chg_10sec.map      = ess.plant.init.pwr_chg_10sec.map;% .*double(ess.plant.init.soc_index <= ess.plant.init.soc_max);
ess.plant.init.curr_chg_burst_max.map = ess.plant.init.curr_chg_burst_max.map;% .*double(ess.plant.init.soc_index <= ess.plant.init.soc_max);
ess.plant.init.pwr_chg_at_soc_init     = interp1(ess.plant.init.soc_index,ess.plant.init.pwr_chg.map,ess.plant.init.soc_init); % (0->1) Power at which battery is charged when SOC is at its minimum
ess.plant.init.pwr_dis_at_soc_init     = interp1(ess.plant.init.soc_index,ess.plant.init.pwr_dis.map,ess.plant.init.soc_init); % (0->1) Power at which battery is discharged when SOC is at its maximum
ess.plant.init.pwr_chg_at_min_soc     = interp1(ess.plant.init.soc_index,ess.plant.init.pwr_chg.map,ess.plant.init.soc_min); % (0->1) Power at which battery is charged when SOC is at its minimum
ess.plant.init.pwr_dis_at_max_soc     = interp1(ess.plant.init.soc_index,ess.plant.init.pwr_dis.map,ess.plant.init.soc_max); % (0->1) Power at which battery is discharged when SOC is at its maximum
%Cell Mass Calculation According to Paul Nelson Equations.
ess.plant.init.a1 = 0.337;
ess.plant.init.b1 = 15.0;
ess.plant.init.c1 = 0.246;
ess.plant.init.mass.cell           = (ess.plant.init.a1*ess.plant.init.pwr_dis_10sec_20soc   + ess.plant.init.b1.*ess.plant.init.cap + ess.plant.init.c1.*(ess.plant.init.a1*ess.plant.init.pwr_dis_10sec_20soc + ess.plant.init.b1.*ess.plant.init.cap).^0.9)/1000;
%[DISCHARGE]  POWER CONSTRAIN PI CONTROLLER
ess.plant.init.ul_pwr_dis         =1 ; %upper limit of the integral of the PI controller for current constrain
ess.plant.init.Ki_pwr_dis         = ess.plant.init.ul_pwr_dis/((ess.plant.init.pwr_dis_10sec_100soc-ess.plant.init.pwr_dis_cont_100soc)/ess.plant.init.pwr_dis_cont_100soc*ess.plant.init.time_burst_max); %integral gain (current)
ess.plant.init.ll_pwr_dis         =-ess.plant.init.time_cont_min*ess.plant.init.Ki_pwr_dis; %lower limit of the integral of the PI controller for current constrain
%[CHARGE]  POWER CONSTRAIN PI CONTROLLER
ess.plant.init.ul_pwr_chg         =1 ; %upper limit of the integral of the PI controller for current constrain
ess.plant.init.Ki_pwr_chg         = ess.plant.init.ul_pwr_chg/((ess.plant.init.pwr_chg_10sec_30soc-ess.plant.init.pwr_chg_cont_30soc)/ess.plant.init.pwr_chg_cont_30soc*ess.plant.init.time_burst_max); %integral gain (current)
ess.plant.init.ll_pwr_chg         =-ess.plant.init.time_cont_min*ess.plant.init.Ki_pwr_dis; %lower limit of the integral of the PI controller for current constrain
% Battery density
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ess.plant.init.pwr_dis_nom        = max((ess.plant.init.volt_nom-ess.plant.init.volt_min_cont).*ess.plant.init.volt_min_cont./(ess.plant.init.rint.map + ess.plant.init.rpol1.map + ess.plant.init.rpol2.map));%per cell
ess.plant.init.pwr_density        = ess.plant.init.pwr_dis_nom/ess.plant.init.mass.cell;
ess.plant.init.energy_density     = (ess.plant.init.volt_nom*ess.plant.init.cap)/ess.plant.init.mass.cell;
%Values should only be used to calculate the number of cells
ess.plant.init.num_cell_series = ess.plant.init.design_num_cell_series;% need to update to make sure we have 0 power at SOC_min
ess.plant.init.num_module_parallel=ess.plant.init.design_num_module_parallel;% need to update to make sure we have 0 power at SOC_min
ess.plant.init.num_cell = ess.plant.init.num_module_parallel.*ess.plant.init.num_cell_series;
ess.plant.init.energy = ess.plant.init.cap.*ess.plant.init.volt_nom;
ess.plant.init.pack_pwr = ess.plant.init.pwr_dis_10sec_20soc*ess.plant.init.num_cell;
ess.plant.init.mass.pack = round(ess.plant.init.mass.cell * ess.plant.init.num_cell);% calculate the mass of the pack
% Battery Total and usable Energy for End of life and Beginning of Life
ess.plant.init.battery_total_erg_eol = ess.plant.init.energy.*ess.plant.init.num_cell;
ess.plant.init.battery_usable_erg_eol = ess.plant.init.energy.*ess.plant.init.num_cell*ess.plant.init.soc_window;
ess.plant.init.battery_total_erg_bol = ess.plant.init.battery_total_erg_eol*ess.plant.init.battery_oversizing_factor_erg;
ess.plant.init.battery_usable_erg_bol = ess.plant.init.battery_usable_erg_eol*ess.plant.init.battery_oversizing_factor_erg;
ess.plant.init.mass.pack = round(ess.plant.init.mass.cell * ess.plant.init.num_cell);% calculate the mass of the pack
clear no_cells_parallel no_cells no_cell_series no_batt E_total_J power_battery mass_battery

