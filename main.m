clc; close all; clearvars;

%% Initialize
addpath('./Exec_Scripts/');
if ~exist("userStruct",'var')
    addpath('./Bez_Functions/');
end

%% sim parameters
model = 'GUAM';

userStruct.variants.scaling = -1; % Scaling factor w.r.t the original VTOL (-1 is default)
userStruct.variants.fmType = 1; % 1=SFunction, 2=Polynomial
userStruct.variants.actType = 4; % 1=None, 2=FirstOrder, 3=SecondOrder, 4=FirstOrderFailSurf
userStruct.variants.propType = 2; % 1=None, 2=FirstOrder, 3=SecondOrder, 4=FirstOrderFailSurf
userStruct.variants.refInputType = 4; % 1=FOUR_RAMP, 2=ONE_RAMP, 3=Timeseries, 4=Piecewise Bezier, 5=Default(doublets)
userStruct.variants.turbType = 1; % 1=Off, 2=Light, 3=Moderate, 4=Severe
% userStruct.variants.ctrlMode = 1; % 1=LocalPilot, 2=RemotePilot, 3=Autonomous
% userStruct.variants.connection.address = '192.168.1.4';

userStruct.switches.AeroPropDeriv = 1; % 1 or 0

userStruct.vehID = 0; % 0=GUAM, 1=HERO

%% initial conditions
% target = struct('tas', 0, 'gndtrack', 0,'stopTime', 30);
PW_Bezier_flag = 0; % Flag to denote failure of PW Bezier setup

kts2fts = 1852.0/(0.3048*3600);

wptsX = [0 0*kts2fts 0]; % within row = pos vel acc
time_wptsX = 0;
wptsY = [0 0 0]; % within row = pos vel acc
time_wptsY = 0;
wptsZ = [0 0 0]; % within row = pos vel acc, NOTE: NED frame -z is up...
time_wptsZ = 0;

% Set desired Initial condition vars
target.RefInput.Vel_bIc_des    = [wptsX(1,2);wptsY(1,2);wptsZ(1,2)];
target.RefInput.pos_des        = [wptsX(1,1);wptsY(1,1);wptsZ(1,1)];
target.RefInput.chi_des        = 0;
target.RefInput.chi_dot_des    = 0;
target.RefInput.trajectory.refTime = [0 max([max(time_wptsX), max(time_wptsY), max(time_wptsZ)])];

% Store the PW Bezier trajectory in the target structure (used in RefInputs)
target.RefInput.Bezier.waypoints = {wptsX, wptsY, wptsZ};
target.RefInput.Bezier.time_wpts = {time_wptsX time_wptsY time_wptsZ};

clear wptsX wptsY wptsZ time_wptsX time_wptsY time_wptsZ 
userStruct.trajFile = ''; % Delete user specified PW Bezier file

%% start the simulation
% set initial conditions
simSetup;
open(model);

% Update trim conditions interpolation to make it compatible with
% simulink
XU0_single = SimIn.Control.trim.XU0_interp;
SimIn.Control.trim.XU0_interp = repmat(XU0_single, 1, 1, 3);
baseW = SimIn.Control.trim.WH;
SimIn.Control.trim.WH = [baseW-1e-6, baseW, baseW+1e-6];

setupSFunction(SimIn,model,false);
SimIn.stopTime = 1000;
open(model);

% SimOut = logsout{1}.Values;
% plotFlightSummary(SimOut, SimIn.Units)

% % Create sample output plots
% simPlots_GUAM;
% % Create an animation of the flight
% Animate_SimOut;