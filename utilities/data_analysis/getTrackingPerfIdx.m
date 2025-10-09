function [T_tilde, Tx, Ty, Tz] = getTrackingPerfIdx(desiredTraj, flownTraj, time, Cnorm)
    % Computes the normalized tracking index T_tilde
    %
    % Inputs:
    %   desiredTraj - [Mx3] trajectory shown to pilot
    %   flownTraj   - [Nx3] actual flown trajectory (same size as time)
    %   time        - [Nx1] time vector
    %   Cnorm       - [1x3] normalization constants [Cx, Cy, Cz]
    %
    % Output:
    %   T_tilde     - scalar normalized tracking index

    % Validate inputs
    assert(size(desiredTraj,2) == 3 && size(flownTraj,2) == 3, 'Trajectories must be Nx3 and Mx3.');
    assert(length(time) == size(flownTraj,1), 'Time vector must match flown trajectory.');
    assert(length(Cnorm) == 3, 'Cnorm must be [Cx, Cy, Cz].');

    N = size(flownTraj,1);
    errorVec = zeros(N, 3);

    % For each point in flownTraj, find the nearest desiredTraj point
    for i = 1:N
        diffs = desiredTraj - flownTraj(i,:);
        d2 = sum(diffs.^2, 2);              % Squared distances
        [~, idx] = min(d2);                 % Index of closest desired point
        errorVec(i,:) = diffs(idx,:);       % Store vector error
    end

    % Square each component
    error2 = errorVec.^2;

    % Total flight time
    Tf = time(end) - time(1);

    % Compute RMS error in each axis
    Tx = sqrt( trapz(time, error2(:,1)) / Tf );
    Ty = sqrt( trapz(time, error2(:,2)) / Tf );
    Tz = sqrt( trapz(time, error2(:,3)) / Tf );

    % Combine
    T_tilde = (1/3) * dot([Tx Ty Tz], 1 ./ Cnorm);
end
