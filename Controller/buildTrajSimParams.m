function [Traj, trajXYZ, duration, OmegaL] = buildTrajSimParams(trajFileName, airspeed, dt, tStart, tEnd)

    if nargin < 4 || isempty(tStart)
        tStart = 0;
    end

    if nargin < 5 || isempty(tEnd)
        tEnd = Inf;
    end

    data = load(fullfile("Trajectories", trajFileName));

    pathPlan = data.SimIn.PathPlan;

    if isfield(pathPlan, "replanTraj") && ~isempty(pathPlan.replanTraj)
        rawTraj = pathPlan.replanTraj;
    elseif isfield(pathPlan, "smoothedTraj") && ~isempty(pathPlan.smoothedTraj)
        rawTraj = pathPlan.smoothedTraj;
    else
        error("buildTrajSimParams:NoTrajectoryFound");
    end

    trajXYZ = rawTraj(:, 1:3);

    [tGrid, stateENU, OmegaL_enu] = buildTrajCore(trajXYZ(:,1), trajXYZ(:,2), trajXYZ(:,3), airspeed, dt);

    % Apply time window [tStart, tEnd]
    idx    = (tGrid >= tStart) & (tGrid <= tEnd);
    tGrid  = tGrid(idx);
    stateENU = stateENU(idx, :);
    OmegaL_enu = OmegaL_enu(idx);

    % ENU(m) to NED(ft)
    stateNED = stateENUm2NEDft(stateENU);     % Nx6 in feet
    OmegaL = -OmegaL_enu;

    % Unpack NED state
    xN = stateNED(:,1);
    yE = stateNED(:,2);
    zD = stateNED(:,3);
    vxN = stateNED(:,4);
    vyE = stateNED(:,5);
    vzD = stateNED(:,6);

    % Build waypoint struct (NED, ft)
    Nout = numel(tGrid);

    Traj.waypointsX = [xN  vxN  zeros(Nout,1)];
    Traj.waypointsY = [yE  vyE  zeros(Nout,1)];
    Traj.waypointsZ = [zD  vzD  zeros(Nout,1)];

    Traj.time_wptsX = tGrid(:).';
    Traj.time_wptsY = tGrid(:).';
    Traj.time_wptsZ = tGrid(:).';

    Traj.OmegaL = OmegaL(:).';

    % Flight duration
    duration = tGrid(end) - tGrid(1);
end
