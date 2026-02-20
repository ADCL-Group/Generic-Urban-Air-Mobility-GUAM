function rp = kinematic_inversion(xyz_req, yaw, g)
    % Extract x and y accelerations
    ddot_x = xyz_req(1);
    ddot_y = xyz_req(2);

    % z acceleration
    ddot_z = xyz_req(3);

    theta_lim = 15*pi/180;
    phi_lim = 30*pi/180;

    den_min = 1e-3;
    den_th = (ddot_z - g);

    if abs(den_th) < den_min
        den_th = sign(den_th) * den_min;
        if den_th == 0
            den_th = den_min;
        end
    end

    % Compute theta_req
    theta_req = atan( (ddot_x*cos(yaw) + ddot_y*sin(yaw)) / den_th );

    den_phi = sqrt(ddot_x^2 + ddot_y^2 + (ddot_z - g)^2);
    if den_phi < den_min
        den_phi = den_min;
    end

    % Compute phi_req
    phi_req = asin( -(ddot_x*sin(yaw) - ddot_y*cos(yaw)) / den_phi );

    % Limit outputs
    phi_req = max(-phi_lim,   min(phi_lim,   phi_req));
    theta_req = max(-theta_lim, min(theta_lim, theta_req));

    rp = [phi_req; theta_req];
end
