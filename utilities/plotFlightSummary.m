function plotFlightSummary(SimOut, Units, maneuverIntervals, PathPlan, plotExtras, vtolState, filePath)
    % plotExtras = true to include additional figures (e.g., controls, euler angles)
    time   = SimOut.Time.Data;

    if nargin < 3 || isempty(maneuverIntervals)
        maneuverIntervals = [time(1), time(end)];
    end
    if nargin < 4 || isempty(PathPlan)
        PathPlan = [];
    end
    if nargin < 5 || isempty(plotExtras)
        plotExtras = true;
    end
    if nargin < 6
        vtolState = [];
    end
    if nargin < 7
        filePath = '';
    end

    havePathPlan = isstruct(PathPlan) && ~isempty(PathPlan);

    alt    = SimOut.Vehicle.Sensor.gpsLLA.Data(:,3) * Units.ft;
    V      = SimOut.Vehicle.Sensor.Vtot.Data / Units.knot;
    psi    = SimOut.Vehicle.Sensor.Euler.psi.Data / Units.deg;
    theta  = SimOut.Vehicle.Sensor.Euler.theta.Data / Units.deg;
    phi    = SimOut.Vehicle.Sensor.Euler.phi.Data / Units.deg;

    prop_om = SimOut.Vehicle.PropAct.EngSpeed.Data * 60 / (2*pi);

    delaL = SimOut.Vehicle.SurfAct.Position.Data(:,1) / Units.deg;
    delaR = SimOut.Vehicle.SurfAct.Position.Data(:,2) / Units.deg;
    deleL = SimOut.Vehicle.SurfAct.Position.Data(:,3) / Units.deg;
    deleR = SimOut.Vehicle.SurfAct.Position.Data(:,4) / Units.deg;
    delr  = SimOut.Vehicle.SurfAct.Position.Data(:,5) / Units.deg;

    Pos_bii = SimOut.Vehicle.EOM.InertialData.Pos_bii.Data / Units.m;
    flowntraj = [ ...
        Pos_bii(:,2), ...   
        Pos_bii(:,1), ...   
        -Pos_bii(:,3)  ...   
    ];

    desiredTraj = [];
    haveDesired = false;

    if isfield(SimOut, 'RTReplan') && ...
       isfield(SimOut.RTReplan, 'flightTrajectory')

        % If this is a timeseries, use .Data; otherwise use it directly.
        ft = SimOut.RTReplan.flightTrajectory;
        if isstruct(ft) && isfield(ft, 'Data')
            desiredTraj = ft.Data(:,1:3);
        else
            desiredTraj = ft(:,1:3);
        end
        haveDesired = ~isempty(desiredTraj);
    % PathPlan.replanTraj
    elseif havePathPlan && isfield(PathPlan, 'replanTraj') && ~isempty(PathPlan.replanTraj)
        desiredTraj = PathPlan.replanTraj(:,1:3);
        haveDesired = true;

    % PathPlan.smoothedTraj
    elseif havePathPlan && isfield(PathPlan, 'smoothedTraj') && ~isempty(PathPlan.smoothedTraj)
        desiredTraj = PathPlan.smoothedTraj(:,1:3);
        haveDesired = true;
    end

    % Crash detection
    crashIdx = [];
    if havePathPlan && isfield(PathPlan, 'result') && ...
            isfield(PathPlan.result, 'maps') && ~isempty(PathPlan.result.maps)

        map = PathPlan.result.maps{1};

        % Try vectorized checkOccupancy; fall back to loop if needed.
        try
            occ = checkOccupancy(map, flowntraj);   % Nx1 if supported
        catch
            occ = zeros(size(flowntraj,1),1);
            for k = 1:size(flowntraj,1)
                occ(k) = checkOccupancy(map, flowntraj(k,:));
            end
        end

        % First time we are inside an occupied cell (crash)
        hit = find(occ > map.FreeThreshold, 1, 'first');
        if ~isempty(hit)
            crashIdx = hit;
        end
    end

    % If a crash was detected, truncate all data and trajectories
    if ~isempty(crashIdx)
        time      = time(1:crashIdx);
        alt       = alt(1:crashIdx);
        V         = V(1:crashIdx);
        psi       = psi(1:crashIdx);
        theta     = theta(1:crashIdx);
        phi       = phi(1:crashIdx);
        prop_om   = prop_om(1:crashIdx,:);
        delaL     = delaL(1:crashIdx);
        delaR     = delaR(1:crashIdx);
        deleL     = deleL(1:crashIdx);
        deleR     = deleR(1:crashIdx);
        delr      = delr(1:crashIdx);
        flowntraj = flowntraj(1:crashIdx,:);
    end

    plotDt = 0.1; % plot every 0.1 s
    simDt  = median(diff(time));
    plotStep = max(1, round(plotDt/simDt));
    idxPlot = 1:plotStep:numel(time);

    maneuverTimes = unique( maneuverIntervals(:) ); 

    % Find the nearest time-index
    getIdx = @(t) find( abs(time - t) == min(abs(time - t)), 1 );

    idxManeuver = arrayfun(@(t) getIdx(t), maneuverTimes);

    if plotExtras
        figure; plot(time(idxPlot), V(idxPlot), 'LineWidth', 1.5); hold on; 
        plot(time(idxManeuver), V(idxManeuver), 'o', 'LineWidth', 1.25); 
        grid on; xlabel('Time (s)'); ylabel('Airspeed (kts)');
        title('Airspeed vs. Time');
        ylim([0, max(V) + 20]);
        local_overlay_vtol_state(time, vtolState);

        figure;
        subplot(3,1,1); plot(time(idxPlot), psi(idxPlot), 'LineWidth', 1.25); hold on; 
        plot(time(idxManeuver), psi(idxManeuver), 'o', 'LineWidth', 1.25); 
        grid on; ylabel('\psi (deg)'); title('Yaw');
        local_overlay_vtol_state(time, vtolState);
        subplot(3,1,2); plot(time(idxPlot), theta(idxPlot), 'LineWidth', 1.25); hold on; 
        plot(time(idxManeuver), theta(idxManeuver), 'o', 'LineWidth', 1.25);
        grid on; ylabel('\theta (deg)'); title('Pitch');
        local_overlay_vtol_state(time, vtolState);
        subplot(3,1,3); plot(time(idxPlot), phi(idxPlot), 'LineWidth', 1.25); hold on; 
        plot(time(idxManeuver), phi(idxManeuver), 'o', 'LineWidth', 1.25);  
        grid on; xlabel('Time (s)'); ylabel('\phi (deg)'); title('Roll');
        local_overlay_vtol_state(time, vtolState);

        figure;
        subplot(3,1,1); plot(time(idxPlot), delaL(idxPlot), time(idxPlot), delaR(idxPlot), 'LineWidth', 1.25); grid on; ylabel('Aileron (deg)'); legend('Left','Right'); title('Aileron Deflections');
        local_overlay_vtol_state(time, vtolState);
        subplot(3,1,2); plot(time(idxPlot), deleL(idxPlot), time(idxPlot), deleR(idxPlot), 'LineWidth', 1.25); grid on; ylabel('Elevator (deg)'); legend('Left','Right'); title('Elevator Deflections');
        local_overlay_vtol_state(time, vtolState);
        subplot(3,1,3); plot(time(idxPlot), delr(idxPlot), 'LineWidth', 1.25); grid on; xlabel('Time (s)'); ylabel('Rudder (deg)'); title('Rudder Deflection');
        local_overlay_vtol_state(time, vtolState);

        figure;
        subplot(2,1,1); plot(time(idxPlot), prop_om(idxPlot,9), 'LineWidth', 1.25); grid on; ylabel('Pusher RPM'); title('Pusher RPM');
        local_overlay_vtol_state(time, vtolState);
        subplot(2,1,2); plot(time(idxPlot), prop_om(idxPlot,1:8), 'LineWidth', 1); grid on; ylabel('Vertical Rotor RPM'); xlabel('Time (s)'); title('Vertical Rotors RPM');
        local_overlay_vtol_state(time, vtolState);
    end

    figure; 
    yyaxis left;
    plot(time(idxPlot), alt(idxPlot), 'LineWidth', 1.5); hold on;
    plot(time(idxManeuver), alt(idxManeuver), 'o', 'LineWidth', 1.25); 
    grid on; xlabel('Time (s)'); ylabel('Altitude (ft)');
    title('Altitude vs. Time');
    local_overlay_vtol_state(time, vtolState);

    % 3D Trajectory
    if haveDesired
        % With map + desired vs flown
        if ~isempty(filePath)
            [~, baseName, ~] = fileparts(filePath);
            fig3D = figure('Name', baseName, 'NumberTitle', 'off');
        else
            figure;
        end
        if havePathPlan && isfield(PathPlan, 'result') && ...
                isfield(PathPlan.result, 'maps') && ~isempty(PathPlan.result.maps)
            show(PathPlan.result.maps{1});
            hold on;
        else
            % If there’s no map object, just proceed without it
            hold on;
        end
        dTraj = plot3(desiredTraj(:,1), desiredTraj(:,2), desiredTraj(:,3), 'r--', 'LineWidth', 2);
        fTraj = plot3(flowntraj(idxPlot,1),  flowntraj(idxPlot,2),  flowntraj(idxPlot,3),  'g-',  'LineWidth', 2);
        scatter3(flowntraj(idxManeuver,1), flowntraj(idxManeuver,2), flowntraj(idxManeuver,3), ...
                 50, 'm','filled','MarkerEdgeColor','k');
        if ~isempty(crashIdx)
            scatter3(flowntraj(end,1), flowntraj(end,2), flowntraj(end,3), ...
                     80, 'r', 'filled', 'MarkerEdgeColor','k');
        end
        grid on; axis equal;
        xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
        title('Desired vs. Flown Trajectory');
        if ~isempty(crashIdx)
            legend([dTraj, fTraj], {'Desired', 'Flown (to crash)'}, 'Location', 'best');
        else
            legend([dTraj, fTraj], {'Desired', 'Flown'}, 'Location', 'best');
        end

        % Distance error over time
        N = size(flowntraj, 1);
        minDist = zeros(N,1);
        % Vectorized nearest distance to desired path
        for i = 1:N
            d2 = sum((desiredTraj - flowntraj(i,:)).^2, 2);
            minDist(i) = sqrt(min(d2));
        end
        figure;
        plot(time(idxPlot), minDist(idxPlot), 'LineWidth', 1.25); hold on;
        plot(time(idxManeuver), minDist(idxManeuver), 'o', 'LineWidth', 1.25);
        yline(30,'r--')
        grid on;
        xlabel('Time (s)'); ylabel('Min Distance to Desired Path [m]');
        ylim([0 max(35, max(minDist) + 5)])
        local_overlay_vtol_state(time, vtolState);
        if ~isempty(crashIdx)
            title('3D Distance from Flown Trajectory to Reference (up to crash)');
        else
            title('3D Distance from Flown Trajectory to Reference');
        end

    else
        % No PathPlan: show flown trajectory only (no map)
        if ~isempty(filePath)
            [~, baseName, ~] = fileparts(filePath);
            fig3D = figure('Name', baseName, 'NumberTitle', 'off');
        else
            figure;
        end
        plot3(flowntraj(idxPlot,1), flowntraj(idxPlot,2), flowntraj(idxPlot,3), 'g-', 'LineWidth', 2); hold on;
        scatter3(flowntraj(idxManeuver,1), flowntraj(idxManeuver,2), flowntraj(idxManeuver,3), ...
                 50, 'm','filled','MarkerEdgeColor','k');
        if ~isempty(crashIdx)
            scatter3(flowntraj(end,1), flowntraj(end,2), flowntraj(end,3), ...
                     80, 'r', 'filled', 'MarkerEdgeColor','k');
        end
        grid on; axis equal;
        xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
        if ~isempty(crashIdx)
            title('Flown Trajectory (Local Frame, up to crash)');
        else
            title('Flown Trajectory (Local Frame)');
        end
        legend('Flown', 'Maneuver Points', 'Location', 'best');
    end

    try
        saveas(fig3D, [baseName '.png']);   % saves in current folder
    catch ME
        % Do nothing
    end

end

function local_overlay_vtol_state(time_ref, vtolState)
% Draw vertical lines + labels where VTOL state changes.
% Compatible with enum, numeric, char/string state signals.

    if isempty(vtolState) || isempty(time_ref)
        return;
    end

    % Try to extract time + state data
    tS = vtolState.Time(:);
    sS = vtolState.Data(:);
    if isempty(tS) || isempty(sS)
        return;
    end

    % Detect change points
    sS = sS(:);
    tS = tS(:);

    % Normalize for comparison
    % For enums: compare directly works. For strings: use string compare.
    if isstring(sS) || ischar(sS)
        sCmp = string(sS);
        chg = [true; sCmp(2:end) ~= sCmp(1:end-1)];
        sLbl = sCmp(chg);
    else
        chg = [true; sS(2:end) ~= sS(1:end-1)];
        sLbl = sS(chg);
    end

    tChg = tS(chg);

    % Vertical lines + labels
    for k = 1:numel(tChg)
        lbl = local_state_to_label(sLbl(k));

        if strcmpi(strtrim(lbl), 'Init')
            continue;
        end

        xline(tChg(k), 'k-', lbl, ...
            'LabelOrientation','aligned', ...
            'LabelVerticalAlignment','top', ...
            'LabelHorizontalAlignment','center', ...
            'HandleVisibility','off');
    end
end

function out = local_state_to_label(s)
% Convert enum/numeric/string to a nice label.
    try
        lbl = char(string(s));
    catch
        try
            lbl = char(s);
        catch
            lbl = 'STATE';
        end
    end
    s = string(lbl);
    s = strtrim(s);
    s = replace(s, "_", " ");
    out = char(s);
end