function [G, v, uMin, uMax, duMin, duMax, Wv, Wu, u_d, du_d] = INDI_control(FM_u, I, m, reference, observed, euler, eta, limits, u_curr, om_prop_bias, alpha, dt)

    varsSize = size(limits,2);
    numEng = varsSize - 4;

%              1 2  3   4  5  6  7  8  9
    propIdx = [7 11 15 19 23 27 31 35 39]; % Index of derivative w.r.t prop speed
    tiltAngleX = [(pi/2) * ones(1, numEng-1), 0]; % Tilt angle w.r.t x body axis
    % tiltAngleZ = deg2rad([0 8 -8 0 0 8 -8 0 90]);  % Tilt angle w.r.t z body axis
    tiltAngleZ = [zeros(1, numEng-1), pi/2];  % Tilt angle w.r.t z body axis

    % Extract data
    pqr_dot_ref = reference(1:3);
    Vz_dot_ref = reference(4);
    V_dot_ref = reference(5);

    pqr_dot_ref(2) = pqr_dot_ref(2) * cos(euler(1));
    pqr_dot_ref(3) = pqr_dot_ref(3) * cos(euler(1))*cos(euler(2));

    % Track the yaw rate r from turn coordination when in fwd flight
    if eta == 0
        pqr_dot_ref(3) = reference(6);
    end

    pqr_dot_o = observed(1:3);
    Vz_dot_o = observed(4);
    V_dot_o = observed(5);

    om_prop_curr = u_curr(1:numEng);
    delta_curr = u_curr(numEng + 1:end);

    om_prop_hi = limits(1, 1:numEng)';
    om_prop_lo = limits(2, 1:numEng)';
    surf_hi    = limits(1, numEng + 1:end)';
    surf_lo    = limits(2, numEng + 1:end)';
    ompdot_max = (100/limits(3,1))*limits(3,1:numEng)'; % Scale it to 230rad/s
    deldot_max = (7/limits(3,numEng + 1)).*limits(3,numEng + 1:end)'; % Scale it to 7rad/s
    % The rate of the control surface must be close to x3wn of the model
    % reference that way the control can keep up with the pilot commands.
    % In a physical instance we should change the wn of the reference model
    % to something slower so the physical actuators can keep up with it

    % Force and moment contributions per propeller input
    Fx_u = FM_u(1, propIdx(1:numEng));
    Fy_u = FM_u(2, propIdx(1:numEng));
    Fz_u = FM_u(3, propIdx(1:numEng));
    dT_domega = sqrt(Fx_u.^2 + Fy_u.^2 + Fz_u.^2); % Total derivative of thrust w.r.t prop speed

    Mx_u = FM_u(4, propIdx(1:numEng));
    My_u = FM_u(5, propIdx(1:numEng));
    Mz_u = FM_u(6, propIdx(1:numEng));

    % Moment Control Allocation (MCA) Matrix
    MCA_prop_M = [Mx_u; My_u; Mz_u]; % Propeller contribution to moments
    MCA_aero_M = [ 0,    FM_u(4,2),         0,         FM_u(4,5);
                   FM_u(5,1),         0,  FM_u(5,4),   0;
                   0, FM_u(6,2),         0,         FM_u(6,5)]; % Aerodynamic contribution to moments for control surfaces

    % Propeller contribution to vertical force
    MCA_prop_F =  [eta 0;0 (1-eta)]*[-dT_domega .* sin(euler(2) + tiltAngleX) .* cos(tiltAngleZ); dT_domega .* cos(alpha + tiltAngleX)];
    
    MCA_prop_M(:,end) = 0;
    MCA_prop_F(1,end) = 0;

    % Zero block for force contribution of control surfaces
    zero_block = zeros(size(MCA_prop_F, 1), size(MCA_aero_M, 2));

    % MCA matrix
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
        % om_prop_max = [zeros(8, 1); 1].*om_prop_hi;
    elseif eta == 1 % Hover
        Wv    = diag([1000, 1000, 1, 100, 100]);
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

    uPhysMin = [om_prop_lo; surf_lo];
    uPhysMax = [om_prop_hi; surf_hi];
    physRange = uPhysMax - uPhysMin;

    Wu = diag(1 ./ physRange);
    Wu(1:numEng-1,1:numEng-1) = Wu(1:numEng-1,1:numEng-1) * 5000;   % punish vertical props 50x more
    Wu(numEng,numEng)    = Wu(numEng,numEng) / 10;        % let pusher move more easily
    Wu(numEng+1:end,numEng+1:end) = Wu(numEng+1:end,numEng+1:end) * 0.5;
    
    du_d = u_d - u_curr;      % "incremental" desired (INDI)
end
