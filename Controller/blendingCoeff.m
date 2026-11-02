function eta = blendingCoeff(vel,  vel_hover, vel_forward)
    % blendingCoeff Computes eta based on airspeed.
    %
    % Inputs:
    %   vel          - Airspeed
    %   vel_hover    - Airspeed at which the vehicle is considered in full hover
    %   vel_forward  - Airspeed at which the vehicle is considered in full forward flight
    %
    % Output:
    %   eta        - Blending coefficient (1 in hover, 0 in forward flight)
    
    % Ensure transition is smooth using a logistic function
    s = (vel - vel_hover) / (vel_forward - vel_hover);
    s = min(max(s, 0), 1);
    
    k = 10; % Steepness factor
    raw = 1 ./ (1 + exp(k * (s - 0.5)));
    
    % Normalize so raw(s=0) -> 1 and raw(s=1) -> 0 without a step jump
    r0 = 1 / (1 + exp(-0.5 * k));
    r1 = 1 / (1 + exp( 0.5 * k));
    eta = (raw - r1) ./ (r0 - r1);
end
