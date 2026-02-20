clc; %close all; clearvars;
setupPath;
setupVariantStruct;
tiltwing = build_vehicle(struct('vehID',-1,'ScalingFactor',1));
trimData = load('Trim_GUAM_XU0_interp.mat', 'XU0_interp');
XU0 = trimData.XU0_interp;

% Get a fwd vel 
uUser = input('Enter u velocity (ft/s): ');
uGrid = XU0(1,:);
uUser = max(min(uUser, uGrid(end)), uGrid(1));

% Interpolate trim conditions
trimXU = interp1(uGrid, XU0.', uUser, 'linear', 'extrap').';

NS = 4; % flap, aileron, elev, rud
NP = 9; % 8 vertical + 1 pusher

[Alon, Blon, Clon, Dlon, Alat, Blat, Clat, Dlat, A, B] = getSSBody(tiltwing, trimXU, NS, NP);

function [Alon, Blon, Clon, Dlon, Alat, Blat, Clat, Dlat, A, B] = getSSBody(tiltwing, trimVector, NS, NP)

rho = 0.0023769; % slugs/ft^3
g = 32.17405; % ft/sec^2

% define some unit vectors, rotations, and skew symmetric matrix functions
e1 = [1; 0; 0];
e2 = [0; 1; 0];
e3 = [0; 0; 1];

Rx = @(x)  [1 0 0 ; 0 cos(x) sin(x); 0 -sin(x) cos(x)];
Ry = @(x)  [cos(x) 0 -sin(x); 0 1 0; sin(x) 0 cos(x)];
Rz = @(x)  [cos(x) sin(x) 0; -sin(x) cos(x) 0; 0 0 1];
hat = @(x) [ 0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];

% Load in the equilibrium state
u0 = trimVector(1); v0 = trimVector(2); w0 = trimVector(3);
p0 = trimVector(4); q0 = trimVector(5); r0 = trimVector(6);
% ax0 = trimVector(7); ay0 = trimVector(8); az0 = trimVector(9);
phi0 = trimVector(10); th0 = trimVector(11); psi0 = trimVector(12);

vb0 = [u0; v0; w0];
om0 = [p0; q0; r0];

% Load in equalibrium controls
surf_idx = 13:13+NS-1;
prop_idx = 13+NS:13+NS+NP-1;
del = trimVector(surf_idx);
omp = trimVector(prop_idx);

tiltwing.del_f = del(1);
tiltwing.del_a = del(2);
tiltwing.del_e = del(3);
tiltwing.del_r = del(4);
tiltwing.om_p  = omp;

% Forces and moments at trim
tiltwing = tiltwing.aero(rho, vb0, om0, true);

FM_x = [ tiltwing.Fx_x; tiltwing.Fy_x; tiltwing.Fz_x;
         tiltwing.Mx_x; tiltwing.My_x; tiltwing.Mz_x ];
FM_u = [ tiltwing.Fx_u; tiltwing.Fy_u; tiltwing.Fz_u;
         tiltwing.Mx_u; tiltwing.My_u; tiltwing.Mz_u ];

F_v = FM_x(1:3,1:3);
F_w = FM_x(1:3,4:6);
M_v = FM_x(4:6,1:3);
M_w = FM_x(4:6,4:6);

% actuator indexing
idx_del = [1 2 4 5]; % flap, ail, elev, rud
idx_omp = [7 11 15 19 23 27 31 35 39]; % prop speeds of each motor

F_u = FM_u(1:3,[idx_del idx_omp]);
M_u = FM_u(4:6,[idx_del idx_omp]);

% mass and inertia
m = tiltwing.mass;
J = tiltwing.I;

% Rotation matrices
Rnb = (Rx(phi0)*Ry(th0)*Rz(psi0));
Rbn = Rnb';                             
g_n = [0;0;g];

% Derivatives of the rotation matrices
Rnb_phi = -Rx(phi0)*hat(e1)*Ry(th0)*Rz(psi0);
Rnb_th  = -Rx(phi0)*Ry(th0)*hat(e2)*Rz(psi0);
Rnb_psi = -Rx(phi0)*Ry(th0)*Rz(psi0)*hat(e3);
Rbn_phi = Rnb_phi';  Rbn_th = Rnb_th'; Rbn_psi = Rnb_psi';

% Euler kinematics matrix
S = [1   sin(phi0)*tan(th0)   cos(phi0)*tan(th0) ; 
     0   cos(phi0)           -sin(phi0)          ; 
     0   sin(phi0)/cos(th0)   cos(phi0)/cos(th0)];

% Derivatives of S wrt the angles
S_phi = [0   cos(phi0)*tan(th0)  -sin(phi0)*tan(th0) ; 
         0  -sin(phi0)           -cos(phi0)          ; 
         0   cos(phi0)/cos(th0)  -sin(phi0)/cos(th0)];

S_th  = [0   sin(phi0)*sec(th0)^2         cos(phi0)*sec(th0)^2 ; 
         0   0                            0                   ; 
         0   sin(phi0)*tan(th0)/cos(th0)  cos(phi0)*tan(th0)/cos(th0)];

Som_eta = [S_phi*om0, S_th*om0, zeros(3,1)];

% State space A and B for x=[pos; euler; v_b; omega]
A = zeros(12,12);
B = zeros(12,NS+NP);
C = eye(12);
D = zeros(12,NS+NP);

% pos dot = Rbn * v_b
A(1:3,4:6) = [Rbn_phi*vb0, Rbn_th*vb0, Rbn_psi*vb0];
A(1:3,7:9) = Rbn;

% euler dot = S * omega
A(4:6,4:6) = Som_eta;
A(4:6,10:12) = S;

% vdot = -omega x v_b + Rnb * g + (1/m)*F
A(7:9,4:6) = [Rnb_phi*g_n, Rnb_th*g_n, Rnb_psi*g_n];
A(7:9,7:9) = -hat(om0) + (1/m)*F_v;
A(7:9,10:12) =  hat(vb0) + (1/m)*F_w;

% omegadot = J \ (M - omega x (J * omega) )
A(10:12,7:9) = J \ M_v;
A(10:12,10:12) = J \ (M_w - hat(om0)*J + hat(J*om0));

% inputs
B(7:9,:) = (1/m)*F_u;
B(10:12,:) = J \ M_u;

x_lon = [7 9 5 11];    % [u w theta q]
y_lon = x_lon;
u_lon = [NS + (1:NP) 3 1]; % [omp1-9 elev flaps]

Alon = A(x_lon,x_lon);
Blon = B(x_lon,u_lon);
Clon = C(y_lon,x_lon);
Dlon = D(y_lon,u_lon);

x_lat = [8 10 12 4]; % [v p r phi]
y_lat = x_lat; 
u_lat = [NS + (1:NP-1) 2 4]; % [omp1-8 ail rud]

Alat = A(x_lat,x_lat);
Blat = B(x_lat,u_lat);
Clat = C(y_lat,x_lat);
Dlat = D(y_lat,u_lat);

end
