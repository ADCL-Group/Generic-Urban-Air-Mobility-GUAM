function [minDistVec, minError, maxError, meanError, perctTunnel] = getTrajectoryError(flownTraj, desiredTraj)
    % Compute minimum 3D Euclidean distance from flown trajectory to desired trajectory
    N = size(flownTraj, 1);
    minDistVec = zeros(N, 1);

    for i = 1:N
        diffs = desiredTraj - flownTraj(i, :);
        d2 = sum(diffs.^2, 2);
        minDistVec(i) = sqrt(min(d2)); % Closest distance
    end

    % Aggregate metrics
    minError = min(minDistVec);
    maxError = max(minDistVec);
    meanError = mean(minDistVec);

    % Percentage of time below 30 m
    perctTunnel = 100 * sum(minDistVec < 30) / length(minDistVec);
end
