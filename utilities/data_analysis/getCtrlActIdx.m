function [C_tilde, C_flap, C_elev, C_rud]   = getCtrlActIdx(surfaceRates, time, Cnorm, isLeftStuck)
    % Computes the control activity index from 5-column rate data
    % Inputs:
    %   rateData       - [Nx5] matrix of surface rates:
    %                    [leftFlap, rightFlap, leftElev, rightElev, rudder]
    %   time           - [Nx1] time vector
    %   Cnorm          - [1x3] normalization constants [C_flap, C_elev, C_rud]
    %   isLeftStuck    - boolean: 1 or 0 is left side is stuck
    %
    % Output:
    %   C_tilde        - scalar control activity index

    % Validate
    assert(size(surfaceRates,2) == 5, 'Expected rateData with 5 columns');
    assert(length(time) == size(surfaceRates,1), 'Time must match number of rows');
    assert(length(Cnorm) == 3, 'Cnorm must be [C_ail, C_elev, C_rud]');

    % Choose flaperon rate based on non-stuck side
    if isLeftStuck
        flapRate = surfaceRates(:,2);  % right flaperon
    else
        flapRate = surfaceRates(:,1);  % left flaperon
    end

    % Average elevator rate
    elevRate = 0.5 * (surfaceRates(:,3) + surfaceRates(:,4));

    % Rudder rate
    rudRate = surfaceRates(:,5);

    % Total time
    Tf = time(end) - time(1);

    % Compute sqrt of integral of abs(rate) per surface
    C_flap = sqrt( trapz(time, abs(flapRate)) / Tf );
    C_elev = sqrt( trapz(time, abs(elevRate)) / Tf );
    C_rud  = sqrt( trapz(time, abs(rudRate))  / Tf );

    % Normalized effort index
    C_tilde = (1/3) * ([C_flap, C_elev, C_rud] * (1 ./ Cnorm(:)));
end
