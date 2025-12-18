function stateNED = stateENUm2NEDft(stateENU)
% Convert ENU [m,m/s] to NED [ft,ft/s].

    m2ft = 3.28084;

    % Allow both 6x1 and Nx6
    [rows, cols] = size(stateENU);
    
    if rows == 6 && cols == 1
        work = stateENU.';      % 1x6
        outAsColumn = true;
    else
        work = stateENU;        % Nx6 or 1x6
        outAsColumn = false;
    end

    xE = work(:,1);
    yN = work(:,2);
    zU = work(:,3);
    vxE = work(:,4);
    vyN = work(:,5);
    vzU = work(:,6);

    % ENU(m) -> NED(m)
    xN_m = yN;
    yE_m = xE;
    zD_m = -zU;

    vxN_m = vyN;
    vyE_m = vxE;
    vzD_m = -vzU;

    % Convert to feet
    workNED = [xN_m yE_m zD_m vxN_m vyE_m vzD_m] * m2ft;

    if outAsColumn
        stateNED = workNED.';   % return 6x1
    else
        stateNED = workNED;     % return Nx6
    end
end
