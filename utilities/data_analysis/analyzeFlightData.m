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

allFlightTables = cell(size(flightCases, 1), 1);
rawTrackingErrors = zeros(size(flightCases, 1), 3); % [RMS_x, RMS_y, RMS_z] per flight
rawControlEfforts = zeros(size(flightCases, 1), 3); % [Cflap, Celev, Crud] per flight

% Settings
weights = [0.6, 0.4]; % [w_T, w_C]

% The Simulation's IC was configured to start approximatedly 10secs before
% the actual start of the trajectory with the same speed and heading. So
% the pilot flies approx for 10s in a straight line and then the tracking
% starts.
trackingStartTime = 10; 

for k = 1:size(flightCases, 1)
    dataFile = flightCases{k, 1};
    maneuverIntervals = flightCases{k, 2};

    load(dataFile);
    % plotFlightSummary(SimOut, PathPlan, Units, true, maneuverIntervals);

    % Flight data
    time = SimOut.Time.Data;
    dt = time(2) - time(1);

    idxTrack = time >= trackingStartTime & time <= maneuverIntervals(end, 2);
    time_trimmed = time(idxTrack);

    surfaceRates = SimOut.Vehicle.SurfAct.Rate.Data(:, [1 2 3 4 5]);

    if isfield(PathPlan, 'replanTraj')
        desiredTraj = PathPlan.replanTraj(:,1:3);
    else
        desiredTraj = PathPlan.smoothedTraj(:,1:3);
    end

    [x, y, z] = latlon2local(...
        SimOut.Vehicle.EOM.WorldRelativeData.LatLonAlt.LatGeod.Data*180/pi, ...
        SimOut.Vehicle.EOM.WorldRelativeData.LatLonAlt.Lon.Data*180/pi, ...
        SimOut.Vehicle.EOM.WorldRelativeData.LatLonAlt.AltGeod.Data/3.281, ...
        PathPlan.result.origin);
    flownTraj = [x, y, z];
    flownTraj_trimmed = flownTraj(idxTrack, :);

    % Failure data
    initTimes = Fail.Surfaces.InitTime;
    aileronInit = initTimes(1:2);
    failIdxAil = find(aileronInit > 0, 1); % failIdxAil is 1 if is left, 2 if is right
    failTime = aileronInit(failIdxAil);
    timeFailIdx = find(time == failTime);
    if ~isempty(timeFailIdx)
        failVal = SimOut.Vehicle.SurfAct.Position.Data(timeFailIdx, failIdxAil) * 180/pi;
    else
        failVal = NaN;
    end

    delaL = SimOut.Vehicle.SurfAct.Position.Data(:,1) * 180/pi;
    delaR = SimOut.Vehicle.SurfAct.Position.Data(:,2) * 180/pi;

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

    % System performance metrics    
    dummyNorm = [1, 1, 1];
    [C_tilde, C_flap, C_elev, C_rud] = getCtrlActIdx(surfaceRates(idxTrack, :), time_trimmed, dummyNorm, failIdxAil == 1);
    [T_tilde, Tx, Ty, Tz] = getTrackingPerfIdx(desiredTraj, flownTraj_trimmed, time_trimmed, dummyNorm);

    rawControlEfforts(k, :) = [C_flap, C_elev, C_rud];
    rawTrackingErrors(k, :) = [Tx, Ty, Tz];

    PI = getPerfIdx(T_tilde, C_tilde, weights);

    % Create summary table with key metrics
    failSide = ["Left", "Right"];
    failAilStr = failSide(failIdxAil);

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

    flightTable = table( ...
        string(dataFile), ...
        failTime, ...
        failVal, ...
        minError, maxError, meanError, perctTunnel, ...
        T_tilde, C_tilde, PI, ...
        {Wx_all}, {Wtot_all}, ...
        mean(Wtot_all), ...
        {fco_lat_all}, {fco_lon_all}, ...
        'VariableNames', { ...
            'File', ...
            'FailTime_s', ...
            'FailedAileron', ...
            'MinTrackErr_m', 'MaxTrackErr_m', ...
            'MeanTrackErr_m', 'PerInside30m', ...
            'T_tilde', 'C_tilde', 'PerformanceIndex' ...
            'PilotAggress_Wx', ...
            'PilotEffort_Wtot', ...
            'PilotEffort_Wtot_Mean', ...
            'CutoffFreq_Lat_Hz', ...
            'CutoffFreq_Long_Hz' ...
        });

    allFlightTables{k} = flightTable;
end

Cnorm_ctrl = max(rawControlEfforts, [], 1);
Cnorm_pos = max(rawTrackingErrors, [], 1);

for k = 1:size(flightCases, 1)
    dataFile = flightCases{k, 1};
    maneuverIntervals = flightCases{k, 2};
    load(dataFile);

    time = SimOut.Time.Data;
    idxTrack = time >= trackingStartTime & time <= maneuverIntervals(end, 2);
    time_trimmed = time(idxTrack);

    surfaceRates = SimOut.Vehicle.SurfAct.Rate.Data(:, [1 2 3 4 5]);

    if isfield(PathPlan, 'replanTraj')
        desiredTraj = PathPlan.replanTraj(:,1:3);
    else
        desiredTraj = PathPlan.smoothedTraj(:,1:3);
    end

    [x, y, z] = latlon2local(...
        SimOut.Vehicle.EOM.WorldRelativeData.LatLonAlt.LatGeod.Data*180/pi, ...
        SimOut.Vehicle.EOM.WorldRelativeData.LatLonAlt.Lon.Data*180/pi, ...
        SimOut.Vehicle.EOM.WorldRelativeData.LatLonAlt.AltGeod.Data/3.281, ...
        PathPlan.result.origin);
    flownTraj = [x, y, z];
    flownTraj_trimmed = flownTraj(idxTrack, :);

    % Failure info
    initTimes = Fail.Surfaces.InitTime;
    aileronInit = initTimes(1:2);
    failIdxAil = find(aileronInit > 0, 1); % failIdxAil is 1 if is left, 2 if is right

    % Recalculate normalized metrics
    C_tilde = getCtrlActIdx(surfaceRates(idxTrack, :), time_trimmed, Cnorm_ctrl, failIdxAil == 1);
    T_tilde = getTrackingPerfIdx(desiredTraj, flownTraj_trimmed, time_trimmed, Cnorm_pos);
    PI = getPerfIdx(T_tilde, C_tilde, weights);

    % Overwrite entries in the table
    allFlightTables{k}.T_tilde = T_tilde;
    allFlightTables{k}.C_tilde = C_tilde;
    allFlightTables{k}.PerformanceIndex = PI;
end

allMetricsTable = vertcat(allFlightTables{:});

