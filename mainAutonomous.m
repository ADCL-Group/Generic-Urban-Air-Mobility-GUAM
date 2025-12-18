clc; close all; clearvars;

addpath('./Exec_Scripts/');
if ~exist("userStruct",'var')
    addpath('./Bez_Functions/');
end

%% sim parameters
model = 'GUAM';

userStruct.variants.fmType = 1; % 1=SFunction, 2=Polynomial
userStruct.variants.actType = 2; % 1=None, 2=FirstOrder, 3=SecondOrder, 4=FirstOrderFailSurf
userStruct.variants.propType = 2; % 1=None, 2=FirstOrder, 3=SecondOrder, 4=FirstOrderFailSurf
userStruct.variants.refInputType = 4; % 1=FOUR_RAMP, 2=ONE_RAMP, 3=Timeseries, 4=Piecewise Bezier, 5=Default(doublets)
userStruct.variants.turbType = 1; % 1=Off, 2=Light, 3=Moderate, 4=Severe

userStruct.variants.ctrlMode = CtrlModeEnum.Autonomous; % 1=LocalPilot, 2=RemotePilot, 3=Autonomous

userStruct.switches.AeroPropDeriv = 1; % 1 or 0

%% path planning
% Snapshot all variables so far
orig = who;

path(genpath('fmt_planner/src'), path);

rng(1,"twister");

result = buildOccupancyMap("fmt_planner/data/osm/boston.osm", 0.1, 20, 35);
origMap = result.maps{1}; 
smoothMap = result.maps{2};
planMap = result.maps{3};
mapLimits = result.mapLimits;

limits = [mapLimits; 0 2*pi];
limits(3,2) = 500;

kts2ms = 1/1.94384001;
MaxRollAngle         = 28*pi/180;
AirSpeed             = 100*kts2ms;
FlightPathAngleLimit = [-5, 5]*pi/180;

flightParams = [MaxRollAngle, AirSpeed, FlightPathAngleLimit];

% [East, North, Up, Heading from East]
start = [ -250 -660 100 pi/2];
goal  = [ -600  600 480 3*pi/2];

g = 9.81;
w = AirSpeed^2/(g*tan(MaxRollAngle));

N = 1000; % number of random samples
rn = optimalRadius(N+2, limits, w); % radius to find neighbors
goalRadius = rn/4; % radius of the goal area around the goal wpt

wpts = [start; goal];

[traj, ~, ~] = FMTWaypoints(planMap, limits, wpts([1 end],:), N, rn, goalRadius, w, flightParams);

% Smooth the trajectory
smoothedTraj = smoothTrajectory( flightParams, limits, smoothMap, traj, wpts, goalRadius);
smoothedTraj(:,4) = computeHeading(smoothedTraj);

figure;
show(origMap); alpha(0.3);
hold on;

hTraj = plot3(smoothedTraj(:,1), smoothedTraj(:,2), smoothedTraj(:,3), 'r-', 'LineWidth', 2);
hStart = scatter3(smoothedTraj(1,1),smoothedTraj(1,2),smoothedTraj(1,3), 100, 'go','filled');
hReached = scatter3(smoothedTraj(end,1),smoothedTraj(end,2),smoothedTraj(end,3), 100, 'mo','filled');
hGoal = scatter3(wpts(end,1), wpts(end,2), wpts(end,3), 150, 'b*','LineWidth',2);

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(-115,60); axis equal
legend([hTraj, hStart, hReached, hGoal], ...
       {'Trajectory','Start','Reached z','Goal'}, ...
       'Location','best');
title(sprintf('FMT* smoothed solution'));
drawnow;

% Store the specified variables in SimIn.PathPlan
SimIn.PathPlan = struct();

SimIn.PathPlan.result             = result;
SimIn.PathPlan.limits             = limits;
SimIn.PathPlan.N                  = N;
SimIn.PathPlan.rn                 = rn;
SimIn.PathPlan.goalRadius         = goalRadius;
SimIn.PathPlan.wpts               = wpts;
SimIn.PathPlan.traj               = traj;
SimIn.PathPlan.smoothedTraj       = smoothedTraj;
SimIn.PathPlan.MaxRollAngle       = MaxRollAngle;
SimIn.PathPlan.AirSpeed           = AirSpeed;
SimIn.PathPlan.FlightPathAngleLimit = FlightPathAngleLimit;

% clear everything except orig + SimIn
keepList = [orig; {'SimIn'}];
clearvars('-except', keepList{:});

%% initial conditions
m2ft = 3.28084; % meters to feet

startIdx = 1;

airspeed = SimIn.PathPlan.AirSpeed * m2ft; % [ft/s]

if isfield(SimIn.PathPlan, "replanTraj") && ~isempty(SimIn.PathPlan.replanTraj)
    trajXYZ = SimIn.PathPlan.replanTraj;
else
    trajXYZ = SimIn.PathPlan.smoothedTraj;
end

% Original trajectory start
orig_xyz = trajXYZ(startIdx, [2 1 3]) * m2ft;
initPos_ft = orig_xyz;

% Nominal heading (in radians from North)
% The heading for the trajectory was computed as 
% atan2(y,x) = atan2(dEast,dNorth) meaning a heading measured from the 
% East axis. The heading in the sim is measured from the north.
heading_rad = mod(pi/2 - trajXYZ(startIdx,4), 2*pi);

% Compute initial velocity vector
vNorth =  airspeed * cos(heading_rad); % [ft/s]
vEast  =  airspeed * sin(heading_rad); % [ft/s]
vDown  = 0;

% First setup initial conditions:
% smoothedTraj is in xEast, yNorth, zUp so we need to switch to 
% xAirplane = yNorth, yAirplane = xEast, zAirplane = -zUp
% pos in ft, vel in ft/sec, and acc in ft/sec^2
wptsX = [initPos_ft(1) vNorth 0]; % within row = pos vel acc
time_wptsX = 0;
wptsY = [initPos_ft(2) vEast 0]; % within row = pos vel acc
time_wptsY = 0;
wptsZ = [-initPos_ft(3) vDown 0]; % within row = pos vel acc, NOTE: NED frame -z is up...
time_wptsZ = 0;

% Set desired Initial condition vars
target.RefInput.Vel_bIc_des    = [wptsX(1,2);wptsY(1,2);wptsZ(1,2)];
target.RefInput.pos_des        = [wptsX(1,1);wptsY(1,1);wptsZ(1,1)];
target.RefInput.chi_des        = heading_rad; % Can be used to set the initial heading
target.RefInput.chi_dot_des    = 0;
target.RefInput.trajectory.refTime = [0 max([max(time_wptsX), max(time_wptsY), max(time_wptsZ)])];

target.RefInput.Bezier.waypoints = {wptsX, wptsY, wptsZ};
target.RefInput.Bezier.time_wpts = {time_wptsX time_wptsY time_wptsZ};

clear wptsX wptsY wptsZ time_wptsX time_wptsY time_wptsZ 
userStruct.trajFile = ''; % Delete user specified PW Bezier file

%% set initial conditions
simSetup;

origin = SimIn.PathPlan.result.origin;
SimIn.IC.LatGeod =  origin(1) * SimIn.Units.deg;
SimIn.IC.Lon = origin(2) * SimIn.Units.deg;

SimIn.stopTime = 1000;        % run sim until end of path

open_system(model);

%% simulation Loop

% Push initial values
flightTrajectory = SimIn.PathPlan.smoothedTraj(:,1:3);
trajLen = size(SimIn.PathPlan.smoothedTraj,1);
resetFlag = 0;

% Execute the model
sim(model);

%% plot

SimOut = logsout{1}.Values;
plotFlightSummary(SimOut, SimIn.Units, [], SimIn.PathPlan, false);

% % % Create an animation of the flight
% % Animate_SimOut;