function [G, v, uMin, uMax, duMin, duMax, Wv, Wu, u_d, du_d] = INDI_control(FM_u, I, m, reference, observed, euler, eta, limits, u_curr, om_prop_bias, alpha, dt)


    propIdx = [7 11 15 19 23 27 31 35 39]; % Index of derivative w.r.t prop speed
    tiltAngleX = [(pi/2) * ones(1, 8), 0]; % Tilt angle w.r.t x body axis
    tiltAngleZ = deg2rad([0 8 -8 0 0 8 -8 0 90]);  % Tilt angle w.r.t z body axis

    % Extract data
    pqr_dot_ref = reference(1:3);
    Vz_dot_ref = reference(4);
    V_dot_ref = reference(5);

    % Track the yaw rate r from turn coordination when in fwd flight
    if eta == 0
        pqr_dot_ref(3) = reference(6);
    end

    pqr_dot_o = observed(1:3);
    Vz_dot_o = observed(4);
    V_dot_o = observed(5);

    om_prop_curr     = u_curr(1:9);
    delta_curr = u_curr(10:end);

    om_prop_hi = limits(1, 1:9)';
    om_prop_lo = limits(2, 1:9)';
    surf_hi    = limits(1, 10:end)';
    surf_lo    = limits(2, 10:end)';
    ompdot_max = limits(3,1:9)';
    deldot_max = 0.5.*limits(3,10:end)';

    % Force and moment contributions per propeller input
    Fx_u = FM_u(1, propIdx);
    Fy_u = FM_u(2, propIdx);
    Fz_u = FM_u(3, propIdx);
    dT_domega = sqrt(Fx_u.^2 + Fy_u.^2 + Fz_u.^2); % Total derivative of thrust w.r.t prop speed

    Mx_u = FM_u(4, propIdx);
    My_u = FM_u(5, propIdx);
    Mz_u = FM_u(6, propIdx);

    % Moment Control Allocation (MCA) Matrix
    MCA_prop_M = [Mx_u; My_u; Mz_u]; % Propeller contribution to moments
    MCA_aero_M = [ 0,    FM_u(4,2),         0,         FM_u(4,5);
                   FM_u(5,1),         0,  FM_u(5,4),   0;
                   0, FM_u(6,2),         0,         FM_u(6,5)]; % Aerodynamic contribution to moments for control surfaces

    % Propeller contribution to vertical force
    MCA_prop_F =  [eta 0;0 (1-eta)]*[-dT_domega .* sin(euler(2) + tiltAngleX) .* cos(tiltAngleZ); dT_domega .* cos(alpha + tiltAngleX)];
    
    % Zero block for force contribution of control surfaces
    zero_block = zeros(size(MCA_prop_F, 1), size(MCA_aero_M, 2));

    % MCA matrix
    % G = [eta * MCA_prop_M, (1 - eta) * MCA_aero_M;
    %      MCA_prop_F,       zero_block];
    G = [MCA_prop_M, MCA_aero_M;
         MCA_prop_F,       zero_block];
    G(isnan(G)) = 0;

    % Compute required force and moments
    Inertia = [I(1,1), 0,       I(1,3);
               0,       I(2,2), 0;
               I(1,3), 0,       I(3,3)];

    req_M = Inertia * (pqr_dot_ref - pqr_dot_o);
    req_F_vert = m * (Vz_dot_ref - Vz_dot_o) / cos(euler(1));
    req_F_fpa = m * (V_dot_ref - V_dot_o);

    % Structured required forces and moments
    v = [req_M; req_F_vert; req_F_fpa];
    
    u_d   = [om_prop_bias; zeros(4, 1)];

    om_prop_min = om_prop_lo;
    om_prop_max = om_prop_hi;
    surf_min = surf_lo;
    surf_max = surf_hi;

    if eta == 0 % Forward flight
        Wv    = diag([1, 1, 1, 1, 1]);
        % duMin = duMin .* [zeros(8, 1); ones(5, 1)]; 
        % duMax = duMax .* [zeros(8, 1); ones(5, 1)]; 
        % uMin = uMin .* [zeros(8, 1); ones(5, 1)]; 
        % uMax = uMax .* [zeros(8, 1); ones(5, 1)]; 
        % u_d   = u_d .* [zeros(8, 1); ones(5, 1)];

        % om_prop_min = [zeros(8, 1); 1].*om_prop_lo;
        % om_prop_max = [zeros(8, 1); 1].*om_prop_hi;
    elseif eta == 1 % Hover
        Wv    = diag([1000, 1000, 1, 100, 100]);
        % duMin = duMin .* [ones(8, 1); zeros(5, 1)]; 
        % duMax = duMax .* [ones(8, 1); zeros(5, 1)]; 
        % uMin = uMin .* [ones(8, 1); zeros(5, 1)]; 
        % uMax = uMax .* [ones(8, 1); zeros(5, 1)]; 
        % u_d   = u_d .* [ones(8, 1); zeros(5, 1)];
        surf_min = 0.*surf_lo;
        surf_max = 0.*surf_hi;
    else % Transition
        Wv    = diag([1000, 1000, 100, 100, 100]);
        surf_min = (1 - eta).*surf_lo;
        surf_max = (1 - eta).*surf_hi;
    end

    duMin = [max(om_prop_min - om_prop_curr,-ompdot_max*dt); max(surf_min - delta_curr, -deldot_max*dt)];
    duMax = [min(om_prop_max - om_prop_curr,ompdot_max*dt); min(surf_max - delta_curr, deldot_max*dt)];
    uMin = [max(om_prop_min, om_prop_curr - ompdot_max*dt); max(surf_min, delta_curr - deldot_max*dt)];
    uMax = [min(om_prop_max, om_prop_curr + ompdot_max*dt); min(surf_max, delta_curr + deldot_max*dt)];

    Wu = diag(1 ./ (uMax - uMin));
    Wu(isnan(Wu) | isinf(Wu)) = 0;
    du_d = u_d - u_curr;      % "incremental" desired (INDI)
end
