function [tiltwing] = build_Hero(~)
%% Tiltwing Aircraft Build Script
% This file is used to specify the tiltwing aircraft 
% configuration. It is called within the MATLAB S-Function
% tiltwing_aero_s.m. The S-Function is used to make an
% tiltwing aerodynamics block within simulink.
% 
% The script produces a "TiltwingClass" class object
% that stores all relevant aircraft information and
% provides a single function to compute the 
% aerodynamic forces and moments based on the
% current aircraft configuration.
%
% The aero function provides the forces and moment
% in the body frame from the velocity, angle of attack,
% side slip angle, and body anglur rates.
% 
% [Fb Mb] = tiltwing.aero(rho, vb, om)

% Modifications:
% 
% 7/20/2018, Jacob Cook: Original script.
% 
% 3-22-2023, MJA: Updated vehicle properties to utilize publicly releasable
% properties found in SACD Lift+Cruise reference configuration at
% https://sacd.larc.nasa.gov/uam-refs/, 
% See Lift+Cruise configuration, NDARC model files for Turbo-electric variant
% Ref. [1] list-cruiseTE6.list
% Ref. [2] lift-cruiseTE6.xlsv
% 2/21/25: John Clardy, added scaling factor


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Makeflyeasy HERO VTOL
%
% Assumptions:
%   - Assume units from GUAM are in English System, also this script.
%     (slug, ft, s.)
%   - Based on OpenVSP model.
% Modifications:
%   - 01/27/2026, MLF: Geometry and mass properties changed and checked wrt
%                      OpenVSP model.
%   - 01/28/2026, MLF: Updated propeller data for VTOL.
%                      APC 15x55MR.
%   - 02/04/2026, MLF: Wing, tail and propeller incidence included. Changes
%                      were made here and in the OpenVSP model due
%                      incidence declared in the HERO VTOL manual.
%   - 02/06/2026, MLF: Mass properties and geometry refined.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Unit conversion variables
kg_to_slug = 0.06852;
m_to_ft = 3.28084;
kgm2_to_slugft2 = 0.73756;

%% Define Scaling Factor
scaling_factor = 1.0;

%% === Fuselage ===
% mass of fuselage
m_b = 0.743 *scaling_factor^3 * kg_to_slug;
% moment of inertia of fuselage
Ib = diag([0.00204 0.04785 0.04837] *scaling_factor^5) * kgm2_to_slugft2;
% body center of mass in the body frame
body_cm_b = [-0.036204; 0; 0.00037] *scaling_factor * m_to_ft;
% cross sectional area
S_b = 0.02055 *scaling_factor^2 * m_to_ft^2;
% planform area
S_p = 0.14845 *scaling_factor^2 * m_to_ft^2;
% wetted area
S_s = 0.45574 *scaling_factor^2 * m_to_ft^2;
% fineness factor, length/max diameter
f_ld = 7.12171;
% Build the body
Body = BodyClass( m_b, ...
                  Ib, ...
                  body_cm_b, ...
                  S_b, ...
                  S_p, ...
                  S_s, ...
                  f_ld);

%% === Wing and Propeller ===
% wing profile
wing_airfoil = readmatrix('wing_NACA3412_geom.txt');
% wing aerodynamic coefficients
wing_aero_coeff = load("NACA_3412_ext.mat");
alf = double(wing_aero_coeff.alpha);
NACA_3412_pp.alf = alf * pi/180;
NACA_3412_pp.cl = spline(alf*pi/180,wing_aero_coeff.Cl);
NACA_3412_pp.cd = spline(alf*pi/180,wing_aero_coeff.Cd);
NACA_3412_pp.cm = spline(alf*pi/180,wing_aero_coeff.Cm);
% wing span and exposed wing span
b = 2.178 *scaling_factor * m_to_ft;
b_e = 2.178 *scaling_factor * m_to_ft;
% chord length
c = [0.24187 0.24187] *scaling_factor * m_to_ft;
% dihedral
gamma = 0 * pi/180;
% wing mass
m_w = 0.7351 *scaling_factor^3 * kg_to_slug;
% moment of inertia
Iw = diag([0.25712 0.00251 0.25951] *scaling_factor^5) * kgm2_to_slugft2;
% wing center of mass in the body frame
w_cm_b = [-0.032853; 0.0; -0.02044] *scaling_factor * m_to_ft;
% wing quarter chord location in the body frame
c4_b = [0.0124; 0.0; -0.0231] *scaling_factor * m_to_ft;
% Wing incidence
i_w = 2.9 * pi/180; 
% aileron location
y_aileron = [0.39945 0.85575] *b/2;
% flap location (not used)
y_flap = [0 0]; % flaperon config: [0.39945 0.85575] *b/2;
% Build the Wing
Wing = WingClass( wing_airfoil, ...
                  NACA_3412_pp, ...
                  [b b_e], ...
                  c, ...
                  gamma,...
                  y_flap, ...
                  y_aileron, ...
                  c4_b, ...
                  m_w, ... 
                  Iw, ...
                  w_cm_b);
% Build the wing-propeller combination
WingProp = WingPropClass(Wing);
WingProp.tilt_angle = i_w;

%% === V-Tail (Vertical) ===
% vtail profile
vtail_airfoil = readmatrix('tail_NACA0009_geom.txt');
% vtail aerodynamic coefficients
vtail_aero_coeff = load("NACA_0009_ext.mat");
alf = double(vtail_aero_coeff.alpha);
NACA_0009_pp.alf = alf * pi/180;
NACA_0009_pp.cl = spline(alf*pi/180,vtail_aero_coeff.Cl);
NACA_0009_pp.cd = spline(alf*pi/180,vtail_aero_coeff.Cd);
NACA_0009_pp.cm = spline(alf*pi/180,vtail_aero_coeff.Cm);
% span 
b_vt = 0.17 *scaling_factor * m_to_ft;
% chord
c_vt = [0.07 0.23] *scaling_factor * m_to_ft;
% v. tail quarter chord location in the body frame
vt_c4_b = [-0.5584; 0; 0.1631] * scaling_factor * m_to_ft;
% mass of v. tail
m_vt = 0.02200 *scaling_factor^3 * kg_to_slug;
% inertia matrix
Ivt = diag([0.00004 0.00008 0.00004]*scaling_factor^5) * kgm2_to_slugft2;
% v. tail center of mass in the body frame
vt_cm_b  = [-0.52088; 0.0; 0.05225] *scaling_factor * m_to_ft;
% rudder location (decompose VTail here?)
y_rudder = [0 0]*b_vt;
% Dihedral angle
gamma_vt = -90 * pi/180;
% Build the vertical tail 
vTail = VerticalTailClass( vtail_airfoil,...
                           NACA_0009_pp,...
                           b_vt, ...
                           c_vt, ...
                           y_rudder, ...
                           vt_c4_b, ...
                           m_vt, ...
                           Ivt, ...
                           vt_cm_b, ...
                           gamma_vt);

%% === V-Tail (Horizontal) ===
% span and exposed span
b_ht = 0.79400 *scaling_factor * m_to_ft;
b_e_ht = 0.72000 *scaling_factor * m_to_ft;
% chord 
c_ht = [0.22000 0.15000] *scaling_factor * m_to_ft;
% dihedral
gamma = -31.5 * pi/180;
% Tail incidence
i_t = 0.5 * pi/180;
% elevator location
y_flap = [0.09091 0.81818]*b_ht/2;
% set the aileron to zero
y_aileron = [0 0];
% inertia matrix
Iht = diag([0.00380 0.00061 0.00370] *scaling_factor^5) * kgm2_to_slugft2;
% h. tail center of mass in the body frame
ht_cm_b = [-0.60855; 0.0; -0.12933] *scaling_factor * m_to_ft;
% h. tail quarter chord location in the body frame
ht_c4_b = [-0.5440; 0.0; -0.0363] *scaling_factor * m_to_ft;
% mass of h. tail
m_ht = 0.10518 *scaling_factor^3 * kg_to_slug;
% Build the horizontal tail
hTail = WingClass( vtail_airfoil, ...
                   NACA_0009_pp, ...
                   [b_ht b_e_ht], ...
                   c_ht, ...
                   gamma,...
                   y_flap, ...
                   y_aileron, ...
                   ht_c4_b, ...
                   m_ht, ...
                   Iht, ...
                   ht_cm_b);

% Build the tail with horizontal and Vertical components
Tail = TailClass(hTail, vTail);
Tail.tilt_angle = i_t;
Tail.TailType = 'Vtail';

%% === Lift and Cruise Propellers ===
% number of props
NP = 5;
% diameter of each propellers
D = [0.381 0.381 0.381 0.381 0.381] *scaling_factor * m_to_ft;
% propeller location in the body frame
p_b = [ 0.34500,  0.34500,  -0.34500, -0.34500, -0.76552;
        -0.35,    0.35,     -0.35,    0.35,     0.0;
        -0.05823, -0.05823, -0.05823, -0.05823, -0.02400] *scaling_factor * m_to_ft;
% motor location in the body frame
m_b =  [ 0.34500,  0.34500,  -0.34500, -0.34500, -0.73552;
        -0.35,     0.35,     -0.35,    0.35,     0.0;
        -0.02823,  -0.02823, -0.02823, -0.02823, -0.02400] *scaling_factor * m_to_ft;
% propeller performance coefficients
scale_prop_coef_15x55MR(scaling_factor);
prop_coefs = load('APC_15x55MR_scaled_coef.mat');
% propeller spin direction (Thrust vector?)
prop_spin = [-1 +1 +1 -1 +1];
% Prop plane angle wrt to wing chord
i_p = 0.0 * pi/180;

% motor mass (Motor + ESC + Prop)
rotor_mass = (0.245+0.068+0.022) *scaling_factor^3 * kg_to_slug; 
rotor_drive_sys_mass = 0.0 *scaling_factor^3 * kg_to_slug;
rotor_engine_mass = 0.0 *scaling_factor^3 * kg_to_slug;
rotor_asbly_mass = rotor_mass + rotor_drive_sys_mass + rotor_engine_mass;

% Pusher (Motor + Prop) (ESC not installed yet)
pusher_mass = (0.245+0.022) *scaling_factor^3 * kg_to_slug;
pusher_drive_sys_mass = 0.0 *scaling_factor^3 * kg_to_slug;
pusher_engine_mass = 0.0 *scaling_factor^3 * kg_to_slug;
pusher_asbly_mass = pusher_mass + pusher_drive_sys_mass + pusher_engine_mass;

m_m = [repmat(rotor_asbly_mass,1,4)  pusher_asbly_mass];

p_T_e = [   -sin(i_p),  -sin(i_p),  -sin(i_p),  -sin(i_p),  1;
            0,  0,  0,  0,  0;
            -1, -1, -1, -1, -sin(-1.77*pi/180)];

% build and array of props
Prop = cell(NP,1);
for ii = 1:NP
  prop = PropellerClass( prop_coefs.APC_15x55MR_scaled_coef,...
                         prop_spin(ii),...
                         D(ii), ...
                         p_b(:,ii), ...
                         m_b(:,ii), ...
                         m_m(ii), ...
                         8880, ... % max rpms
                         p_T_e(:,ii));
   Prop{ii} = prop;
end

%% === Point masses ===
% Booms_R
booms_R_mass = 0.08 *scaling_factor^3 * kg_to_slug;
booms_R_I = diag([0 0 0]*scaling_factor^5) * kgm2_to_slugft2;
booms_R_cm_b = [0.000; 0.3500; 0.0018] *scaling_factor * m_to_ft;
Booms_R = MassClass(booms_R_mass, booms_R_I, booms_R_cm_b);

% Booms_L
booms_L_mass = 0.08 *scaling_factor^3 * kg_to_slug;
booms_L_I = diag([0 0 0]*scaling_factor^5) * kgm2_to_slugft2;
booms_L_cm_b = [0.000; -0.3500; 0.0018] *scaling_factor * m_to_ft;
Booms_L = MassClass(booms_L_mass, booms_L_I, booms_L_cm_b);

% Battery
batt_mass = 1.9 *scaling_factor^3 * kg_to_slug;
batt_I = diag([0 0 0]*scaling_factor^5) * kgm2_to_slugft2;
batt_cm_b = [0.16782; 0.0; 0.03513] *scaling_factor * m_to_ft;
Batt = MassClass(batt_mass, batt_I, batt_cm_b);

% Avionics
avionics_mass = 0.8 *scaling_factor^3 * kg_to_slug;
avionics_I = diag([0 0 0]*scaling_factor^5) * kgm2_to_slugft2;
avionics_cm_b = [0.0; 0.0; 0.0] *scaling_factor * m_to_ft;
Avionics = MassClass(avionics_mass, avionics_I, avionics_cm_b);

Extra_Mass = {Booms_R Booms_L Batt Avionics};

%% === Tiltwing Aircraft ===
tiltwing = TiltWingClass( WingProp, Tail, Body, Prop, Extra_Mass);

