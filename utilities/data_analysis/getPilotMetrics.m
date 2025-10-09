function [Wx, Wtot, wco_lon, fco_lon, wco_lat, fco_lat, wco_dir, fco_dir] = getPilotMetrics(PilotInputs, dt, tStart, tEnd)
    % Compute pilot aggressiveness and cut-off metrics during a maneuver
    %
    % Inputs:
    %   PilotInputs - Simulink struct of time and signals (Nx6)
    %   dt          - sample time [s]
    %   tStart      - start time of maneuver [s]
    %   tEnd        - end time of maneuver [s]
    %
    % Outputs:
    %   Wx      - 1×5 vector of pilot aggressiveness per axis (Lat, Lon, Dir, Thr, Col)
    %   Wtot    - total aggressiveness (%)
    %   wco_lon, fco_lon - cutoff freq (rad/s and Hz) for longitudinal stick
    %   wco_lat, fco_lat - cutoff freq (rad/s and Hz) for lateral stick

    % Extract pilot input signals
    t    = PilotInputs.time;
    dLat = PilotInputs.signals.values(:,1);           % lateral stick
    dLon = PilotInputs.signals.values(:,2);           % longitudinal stick
    dDir = PilotInputs.signals.values(:,3);           % directional pedal
    dThr = mean(PilotInputs.signals.values(:, 4:5), 2); % 2 throttles --> average
    dCol = PilotInputs.signals.values(:,6);           % collective

    signals = [dLat, dLon, dDir, dThr, dCol];

    % Get the trim conditions (first second of flight)
    TRIM_WINDOW = 1.0;
    idxTrim     = t <= TRIM_WINDOW;
    trimVals    = mean(signals(idxTrim, :), 1);

    % Select maneuver interval
    idxMet     = (t >= tStart) & (t <= tEnd);
    tMet       = t(idxMet);
    signalsMet = signals(idxMet, :);

    % Aggressiveness
    deltaRange  = [2 2 2 2 2];  % All inceptors range from -1 to 1
    f   = abs(signalsMet - trimVals) ./ deltaRange;
    Wx  = 100/(tMet(end)-tMet(1)) * sum(f,1) * dt;
    Wtot = sum(Wx);

    % Cutoff frequencies
    [wco_lon, fco_lon] = pilotCutOff(dLon(idxMet), dt);
    [wco_lat, fco_lat] = pilotCutOff(dLat(idxMet), dt);
    [wco_dir, fco_dir] = pilotCutOff(dDir(idxMet), dt);
end
