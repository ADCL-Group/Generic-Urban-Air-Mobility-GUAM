function plotFlightSummary(SimOut, Units, maneuverIntervals, PathPlan, plotExtras)
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

    havePathPlan = isstruct(PathPlan) && ~isempty(PathPlan);

    V      = SimOut.Vehicle.Sensor.Vtot.Data / Units.knot;
    psi    = SimOut.Vehicle.Sensor.Euler.psi.Data * 180/pi;
    theta  = SimOut.Vehicle.Sensor.Euler.theta.Data * 180/pi;
    phi    = SimOut.Vehicle.Sensor.Euler.phi.Data * 180/pi;

    prop_om = SimOut.Vehicle.PropAct.EngSpeed.Data * 60 / (2*pi);

    delaL = SimOut.Vehicle.SurfAct.Position.Data(:,1) * 180/pi;
    delaR = SimOut.Vehicle.SurfAct.Position.Data(:,2) * 180/pi;
    deleL = SimOut.Vehicle.SurfAct.Position.Data(:,3) * 180/pi;
    deleR = SimOut.Vehicle.SurfAct.Position.Data(:,4) * 180/pi;
    delr  = SimOut.Vehicle.SurfAct.Position.Data(:,5) * 180/pi;

    lat_deg = SimOut.Vehicle.EOM.WorldRelativeData.LatLonAlt.LatGeod.Data * 180/pi;
    lon_deg = SimOut.Vehicle.EOM.WorldRelativeData.LatLonAlt.Lon.Data     * 180/pi;
    alt_m   = SimOut.Vehicle.EOM.WorldRelativeData.LatLonAlt.AltGeod.Data / 3.281;

    if havePathPlan && isfield(PathPlan,'result') && isfield(PathPlan.result,'origin')
        origin = PathPlan.result.origin;
        [x,y,z] = latlon2local(lat_deg, lon_deg, alt_m, origin);
        flowntraj = [x, y, z];

        % desired trajectory (replan if present, else smoothed)
        if isfield(PathPlan, 'replanTraj')
            desiredTraj = PathPlan.replanTraj(:,1:3);
        else
            desiredTraj = PathPlan.smoothedTraj(:,1:3);
        end
        haveDesired = ~isempty(desiredTraj);
    else
        % No PathPlan: build a local frame from the first sample; no desired path.
        origin = [lat_deg(1), lon_deg(1), alt_m(1)];
        [x,y,z] = latlon2local(lat_deg, lon_deg, alt_m, origin);
        flowntraj = [x, y, z];
        desiredTraj = [];
        haveDesired = false;
    end

    maneuverTimes = unique( maneuverIntervals(:) ); 

    % Find the nearest time-index
    getIdx = @(t) find( abs(time - t) == min(abs(time - t)), 1 );

    idxManeuver = arrayfun(@(t) getIdx(t), maneuverTimes);

    timeIdx = zeros(size(maneuverIntervals,1),1);
    for i=1:size(maneuverIntervals,1)
        timeIdx(i) = find(time == maneuverIntervals(i,1));
    end

    if plotExtras
        figure; plot(time, V, 'LineWidth', 1.5);
        grid on; xlabel('Time (s)'); ylabel('Airspeed (kts)');
        title('Airspeed vs. Time');

        figure;
        subplot(3,1,1); plot(time, psi, 'LineWidth', 1.25); grid on; ylabel('\psi (deg)'); title('Yaw');
        subplot(3,1,2); plot(time, theta, 'LineWidth', 1.25); grid on; ylabel('\theta (deg)'); title('Pitch');
        subplot(3,1,3); 
        plot(time, phi, 'LineWidth', 1.25); hold on; 
        plot(time(idxManeuver), phi(idxManeuver), 'o', 'LineWidth', 1.25);  
        grid on; xlabel('Time (s)'); ylabel('\phi (deg)'); title('Roll');

        figure;
        subplot(3,1,1); plot(time, delaL, time, delaR, 'LineWidth', 1.25); grid on; ylabel('Aileron (deg)'); legend('Left','Right'); title('Aileron Deflections');
        subplot(3,1,2); plot(time, deleL, time, deleR, 'LineWidth', 1.25); grid on; ylabel('Elevator (deg)'); legend('Left','Right'); title('Elevator Deflections');
        subplot(3,1,3); plot(time, delr, 'LineWidth', 1.25); grid on; xlabel('Time (s)'); ylabel('Rudder (deg)'); title('Rudder Deflection');

        figure;
        subplot(2,1,1); plot(time, prop_om(:,9), 'LineWidth', 1.25); grid on; ylabel('Pusher RPM'); title('Pusher RPM');
        subplot(2,1,2); plot(time, prop_om(:,1:8), 'LineWidth', 1); grid on; ylabel('Vertical Rotor RPM'); xlabel('Time (s)'); title('Vertical Rotors RPM');
    end

    % 3D Trajectory
    if haveDesired
        % With map + desired vs flown
        figure;
        if isfield(PathPlan.result, 'maps') && ~isempty(PathPlan.result.maps)
            show(PathPlan.result.maps{1});
            hold on;
        else
            % If there’s no map object, just proceed without it
            hold on;
        end
        dTraj = plot3(desiredTraj(:,1), desiredTraj(:,2), desiredTraj(:,3), 'r--', 'LineWidth', 2);
        fTraj = plot3(flowntraj(:,1),  flowntraj(:,2),  flowntraj(:,3),  'g-',  'LineWidth', 2);
        scatter3(flowntraj(idxManeuver,1), flowntraj(idxManeuver,2), flowntraj(idxManeuver,3), ...
                 50, 'm','filled','MarkerEdgeColor','k');
        grid on; axis equal;
        xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
        title('Desired vs. Flown Trajectory');
        legend([dTraj, fTraj], {'Desired', 'Flown'}, 'Location', 'best');

        % Distance error over time
        N = size(flowntraj, 1);
        minDist = zeros(N,1);
        % Vectorized nearest distance to desired path
        for i = 1:N
            d2 = sum((desiredTraj - flowntraj(i,:)).^2, 2);
            minDist(i) = sqrt(min(d2));
        end
        figure;
        plot(time, minDist, 'LineWidth', 1.25); hold on;
        plot(time(idxManeuver), minDist(idxManeuver), 'o', 'LineWidth', 1.25);
        grid on;
        xlabel('Time (s)'); ylabel('Min Distance to Desired Path [m]');
        title('3D Distance from Flown Trajectory to Reference');

    else
        % No PathPlan: show flown trajectory only (no map)
        figure;
        plot3(flowntraj(:,1), flowntraj(:,2), flowntraj(:,3), 'g-', 'LineWidth', 2); hold on;
        scatter3(flowntraj(idxManeuver,1), flowntraj(idxManeuver,2), flowntraj(idxManeuver,3), ...
                 50, 'm','filled','MarkerEdgeColor','k');
        grid on; axis equal;
        xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
        title('Flown Trajectory (Local Frame)');
        legend('Flown', 'Maneuver Points', 'Location', 'best');
    end
end
