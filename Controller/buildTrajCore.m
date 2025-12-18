function [tGrid, stateENU, OmegaL] = buildTrajCore(x, y, z, v, dt)
% Core ENU(m) trajectory generation
%   Inputs:
%       x,y,z : waypoints in ENU [m] (any shape, will be vectorized)
%       v     : desired constant speed [m/s]
%       dt    : sample period [s]
%
%   Outputs:
%       tGrid    : Nx1 time vector [s]
%       stateENU : Nx6 matrix [x y z vx vy vz] in ENU, [m] and [m/s]

    % Force into column vectors
    x = x(:); y = y(:); z = z(:);
    N = numel(x);

    % Degenerate / invalid case handling
    if N == 0
        x0 = 0; y0 = 0; z0 = 0;
        tGrid = 0;
        stateENU = [x0 y0 z0 0 0 0];
        OmegaL = 0;
        return;
    end

    if (N < 2) || (v <= 0) || (dt <= 0)
        % Static point at the first waypoint
        x0 = x(1); y0 = y(1); z0 = z(1);
        tGrid = 0;
        stateENU = [x0 y0 z0 0 0 0];
        OmegaL = 0;
        return;
    end

    % Arc length along the 3D path
    dx = diff(x); dy = diff(y); dz = diff(z);
    segLen = sqrt(dx.^2 + dy.^2 + dz.^2);
    sNodes = [0; cumsum(segLen)];
    sEnd   = sNodes(end);

    if sEnd <= 0
        % All waypoints at same position
        x0 = x(1); y0 = y(1); z0 = z(1);
        tGrid = 0;
        stateENU = [x0 y0 z0 0 0 0];
        OmegaL = 0;
        return;
    end

    % Uniform s-grid (constant speed)
    ds    = v * dt;
    sGrid = (0:ds:sEnd).';
    tGrid = sGrid / v;

    % Interpolate positions vs s
    x_s = interp1(sNodes, x, sGrid, 'linear');
    y_s = interp1(sNodes, y, sGrid, 'linear');
    z_s = interp1(sNodes, z, sGrid, 'linear');

    % Derivatives wrt s to tangent
    dxds = gradient(x_s, sGrid);
    dyds = gradient(y_s, sGrid);
    dzds = gradient(z_s, sGrid);
    speed_s = sqrt(dxds.^2 + dyds.^2 + dzds.^2);

    % Protect against division by zero
    speed_s(speed_s == 0) = eps;

    % Normalize tangent
    tx = dxds ./ speed_s;
    ty = dyds ./ speed_s;
    tz = dzds ./ speed_s;

    % Velocity = v * tangent (enforce constant speed)
    vx = v * tx;
    vy = v * ty;
    vz = v * tz;

    % Pack ENU state [x y z vx vy vz]
    stateENU = [x_s y_s z_s vx vy vz];

    % OmegaL computation
    % heading_enu = atan2(vy, vx); % 0 = East, ccw to North
    % OmegaL = gradient(heading_enu, tGrid); % d(heading)/dt [rad/s]
    ax = gradient(vx, tGrid);
    ay = gradient(vy, tGrid);

    vxy2 = vx.^2 + vy.^2;
    small = vxy2 < 1e-6;

    OmegaL = (vx .* ay - vy .* ax) ./ vxy2;
    OmegaL(small) = 0;
end
