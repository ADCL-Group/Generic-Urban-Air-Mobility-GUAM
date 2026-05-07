clc; clear; close all;

% Define all flight files and their respective maneuver intervals
flightCases = {
    % Nominal without failure
    % 'flight_summary39.mat', [10 17.1; 17.1 30; 30 40; 40 48.9];
    'flight_summary40.mat', [10 16; 16 30; 30 36; 36 40; 40 47];
    % 'flight_summary41.mat', [10 16; 16 29.5; 29.5 40; 40 48.1];
    % 'flight_summary42.mat', [10 20; 20 40; 40 52; 52 60];

    % Replanned
    'flight_summary43.mat', [10 20; 20 40];
    'flight_summary44.mat', [10 20; 20 40; 40 55];
    'flight_summary45.mat', [10 20; 20 40; 40 57];
    'flight_summary46.mat', [10 66; 66 98; 98 106];
    'flight_summary47.mat', [13 76; 76 93; 93 120];
    'flight_summary49.mat', [16.7 40.3; 40.3 61.5; 61.5 136; 136 194];
    'flight_summary50.mat', [16.7 40; 40 52.7; 52.7 136.9; 136.9 148.9; 148.9 214.8];
    'flight_summary51.mat', [12 37.9; 37.9 48.5; 48.5 57.5; 57.5 135.3; 135.3 218];
    'flight_summary52.mat', [15.4 40; 40 54.3; 54.3 61; 61 138.5; 138.5 217];

    % Nominal with failure
    'flight_summary16.mat', [10 16.6; 16.6 29; 29 40; 40 48];
    'flight_summary14.mat', [10 17.94; 17.94 31; 31 41; 41 50];
    'flight_summary15.mat', [10 17; 17 30; 30 40; 40 48];
};

nFlights = size(flightCases, 1);

allFlightTables = cell(nFlights, 1);
rawTrackingErrors = zeros(nFlights, 3); % [RMS_x, RMS_y, RMS_z] per flight
rawControlEfforts = zeros(nFlights, 3); % [Cflap, Celev, Crud] per flight
rawWtot           = zeros(nFlights, 1); % Pilot effort mean

% Cache data needed for normalized pass:
time_trimmed_all     = cell(nFlights, 1);
desiredTraj_all      = cell(nFlights, 1);
surfaceRates_all     = cell(nFlights, 1);
flownTraj_trimmed_all= cell(nFlights, 1);
failIdxAil_all       = cell(nFlights, 1);

% Settings
weights = [0.6, 0.4]; % [w_T, w_E]

% The Simulation's IC was configured to start approximatedly 10secs before
% the actual start of the trajectory with the same speed and heading. So
% the pilot flies approx for 10s in a straight line and then the tracking
% starts.
trackingStartTime = 10; 

for k = 1:nFlights
    dataFile = flightCases{k, 1};
    maneuverIntervals = flightCases{k, 2};

    % Detect crash from maneuverIntervals == [0 Inf]
    isCrash = (size(maneuverIntervals,1) == 1) && ...
              (maneuverIntervals(1,1) == 0) && ...
              isinf(maneuverIntervals(1,2));

    load(dataFile);
    % plotFlightSummary(SimOut, Units, [], PathPlan, false, flightCases{k, 1});
    % plotFlightSummary(SimOut, Units, maneuverIntervals, PathPlan, false);

    % Flight data
    time = SimOut.Time.Data;
    dt = time(2) - time(1);

    idxTrack = time >= trackingStartTime & time <= maneuverIntervals(end, 2);
    time_trimmed = time(idxTrack);
    time_trimmed_all{k} = time_trimmed;   % cache

    surfaceRates = SimOut.Vehicle.SurfAct.Rate.Data(:, [1 2 3 4 5]);
    surfaceRates_all{k} = surfaceRates(idxTrack, :);

    if isfield(SimOut, 'RTReplan') && ...
       isfield(SimOut.RTReplan, 'flightTrajectory')

        % If this is a timeseries, use .Data; otherwise use it directly.
        ft = SimOut.RTReplan.flightTrajectory;
        if isstruct(ft) && isfield(ft, 'Data')
            desiredTraj = ft.Data(:,1:3);
        else
            desiredTraj = ft(:,1:3);
        end
    % PathPlan.replanTraj
    elseif isfield(PathPlan, 'replanTraj') && ~isempty(PathPlan.replanTraj)
        desiredTraj = PathPlan.replanTraj(:,1:3);
    % PathPlan.smoothedTraj
    elseif isfield(PathPlan, 'smoothedTraj') && ~isempty(PathPlan.smoothedTraj)
        desiredTraj = PathPlan.smoothedTraj(:,1:3);
    end
    desiredTraj_all{k} = desiredTraj;     % cache

    Pos_bii = SimOut.Vehicle.EOM.InertialData.Pos_bii.Data / 3.281;
    flownTraj = [Pos_bii(:,2), Pos_bii(:,1), -Pos_bii(:,3)];
    flownTraj_trimmed = flownTraj(idxTrack, :);
    flownTraj_trimmed_all{k} = flownTraj_trimmed;  % cache

    % Failure data
    failIdxAil = 1; 

    if exist('Fail', 'var') && isfield(Fail, 'Surfaces')
        initTimes = Fail.Surfaces.InitTime;
        aileronInit = initTimes(1:2);
        foundIdx = find(aileronInit > 0, 1);
        
        if ~isempty(foundIdx)
            failIdxAil = foundIdx;
        end
    end
    
    failIdxAil_all{k} = failIdxAil;

    % Trajectory tracking metrics
    [minDistVec, minError, maxError, meanError, perctTunnel] = getTrajectoryError(flownTraj_trimmed, desiredTraj);

    % Pilot effort and aggressiveness metrics
    nManeuvers = size(maneuverIntervals, 1);
    pilotMetrics(nManeuvers) = struct( ...
        'interval', [], ...
        'Wx', [], ...
        'Wtot', [], ...
        'wco_lon', [], 'fco_lon', [], ...
        'wco_lat', [], 'fco_lat', [], ...
        'wco_dir', [], 'fco_dir', []);

    for i = 1:size(maneuverIntervals, 1)
        tStart = maneuverIntervals(i, 1);
        tEnd   = maneuverIntervals(i, 2);
        [Wx, Wtot, wco_lon, fco_lon, wco_lat, fco_lat, wco_dir, fco_dir] = ...
            getPilotMetrics(PilotInputs, dt, tStart, tEnd);

        pilotMetrics(i).interval = [tStart tEnd];
        pilotMetrics(i).Wx       = Wx;
        pilotMetrics(i).Wtot     = Wtot;
        pilotMetrics(i).wco_lon  = wco_lon;
        pilotMetrics(i).fco_lon  = fco_lon;
        pilotMetrics(i).wco_lat  = wco_lat;
        pilotMetrics(i).fco_lat  = fco_lat;
        pilotMetrics(i).wco_dir  = wco_dir;
        pilotMetrics(i).fco_dir  = fco_dir;
    end

    Wx_all   = cell(nManeuvers, 1);
    Wtot_all = zeros(nManeuvers, 1);
    fco_lon_all = zeros(nManeuvers, 1);
    fco_lat_all = zeros(nManeuvers, 1);

    for i = 1:nManeuvers
        Wx_all{i}   = pilotMetrics(i).Wx;
        Wtot_all(i) = pilotMetrics(i).Wtot;
        fco_lon_all(i) = pilotMetrics(i).fco_lon;
        fco_lat_all(i) = pilotMetrics(i).fco_lat;
    end
    rawWtot(k) = mean(Wtot_all);

    % System performance metrics    
    dummyNorm = [1, 1, 1];
    [A_tilde_raw, A_flap, A_elev, A_rud] = getCtrlActIdx(surfaceRates_all{k}, time_trimmed, dummyNorm, failIdxAil == 1);
    [T_tilde_raw, Tx, Ty, Tz] = getTrackingPerfIdx(desiredTraj, flownTraj_trimmed, time_trimmed, dummyNorm);

    rawControlEfforts(k, :) = [A_flap, A_elev, A_rud];
    rawTrackingErrors(k, :) = [Tx, Ty, Tz];

    E_tilde_raw = rawWtot(k);

    PI_raw = getPerfIdx(T_tilde_raw, E_tilde_raw, weights);

    % Create summary table with key metrics
    failSide = ["Left", "Right"];
    failAilStr = failSide(failIdxAil);

    flightTable = table( ...
        string(dataFile), ...
        isCrash, ...
        minError, maxError, meanError, perctTunnel, ...
        T_tilde_raw, A_tilde_raw, E_tilde_raw, PI_raw, ...
        {Wx_all}, {Wtot_all}, ...
        rawWtot(k), ...
        {fco_lat_all}, {fco_lon_all}, ...
        'VariableNames', { ...
            'File', ...
            'IsCrash', ...
            'MinTrackErr_m', 'MaxTrackErr_m', ...
            'MeanTrackErr_m', 'PerInside30m', ...
            'T_tilde', 'A_tilde', 'E_tilde', 'PerformanceIndex', ...
            'PilotAggress_Wx', ...
            'PilotEffort_Wtot', ...
            'PilotEffort_Wtot_Mean', ...
            'CutoffFreq_Lat_Hz', ...
            'CutoffFreq_Long_Hz' ...
        });

    allFlightTables{k} = flightTable;
end

% Compute global normalization constants
Cnorm_efft = max(rawWtot); % pilot effort max
Cnorm_ctrl = max(rawControlEfforts, [], 1);% [max Af, max Ae, max Ar]
Cnorm_pos  = max(rawTrackingErrors, [], 1);% [max Tx, max Ty, max Tz]

for k = 1:nFlights
    % Recalculate normalized metrics
    E_tilde = rawWtot(k) / Cnorm_efft;
    A_tilde = getCtrlActIdx(surfaceRates_all{k}, time_trimmed_all{k}, Cnorm_ctrl, failIdxAil_all{k} == 1);
    T_tilde = getTrackingPerfIdx(desiredTraj_all{k}, flownTraj_trimmed_all{k}, time_trimmed_all{k}, Cnorm_pos);
    PI = getPerfIdx(T_tilde, E_tilde, weights);

    % Overwrite entries in the table
    allFlightTables{k}.T_tilde = T_tilde;
    allFlightTables{k}.A_tilde = A_tilde;
    allFlightTables{k}.E_tilde = E_tilde;
    allFlightTables{k}.PerformanceIndex = PI;
end

allMetricsTable = vertcat(allFlightTables{:});

