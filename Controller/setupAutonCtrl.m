function Out = setupAutonCtrl(SimIn)
% ADCL AUTONOMOUS GUIDANCE PARAMETERS

% ------------------------------------------------------------
% Fixed Wing Cruise (TECS)
% ------------------------------------------------------------

% Maximum sink rate
% PX4 Parameter: FW_T_SINK_MAX
% PX4 Default: 5 m/s
Out.FW.TECS.max_sink_rate = 5 * SimIn.Units.m;

% Minimum sink rate
% PX4 Parameter: FW_T_SINK_MIN
% PX4 Default: 2 m/s
Out.FW.TECS.min_sink_rate = 2 * SimIn.Units.m;

% Maximum climb rate
% PX4 Parameter: FW_T_CLMB_MAX
% PX4 Default: 5 m/s
Out.FW.TECS.climb_rate = 5 * SimIn.Units.m;

% Fast descend mode
% PX4 internal TECS flag
Out.FW.TECS.fast_descend = 0;

% Altitude control time constant
% PX4 Parameter: FW_T_ALT_TC
% PX4 Default: 5 s
Out.FW.TECS.Tv = 3;

% Altitude loop gain
Out.FW.TECS.Kh = 1 / Out.FW.TECS.Tv;

% True airspeed error time constant
% PX4 Parameter: FW_T_TAS_TC
% PX4 Default: 5 s
Out.FW.TECS.Kv = 0.2;   % 1 / FW_T_TAS_TC

% Height rate feedforward
% PX4 Parameter: FW_T_HRATE_FF
% PX4 Default: 0.3
Out.FW.TECS.Khff = 0.3;

% Pitch integrator gain
% PX4 Parameter: FW_T_I_GAIN_PIT
% PX4 Default: 0.1
Out.FW.TECS.Ki = 0.1;

% Pitch damping gain
% PX4 Parameter: FW_T_PTCH_DAMP
% PX4 Default: 0.1
Out.FW.TECS.K = 0.1;

% Specific energy balance feedforward
% PX4 Parameter: FW_T_SEB_R_FF
% PX4 Default: 1.0
Out.FW.TECS.Kff = 1;

% Pitch limits
Out.FW.TECS.max_pitch = 28 * SimIn.Units.deg;
Out.FW.TECS.min_pitch = -28 * SimIn.Units.deg;

% ------------------------------------------------------------
% Throttle dynamics

Out.FW.TECS.throttle_max = 1;
Out.FW.TECS.throttle_min = 0;

% Throttle damping
% PX4 Parameter: FW_T_THR_DAMPING
% PX4 Default: 0.05
Out.FW.TECS.thr_damping = 0.05;

% Throttle integrator gain
% PX4 Parameter: FW_T_THR_INTEG
% PX4 Default: 0.02
Out.FW.TECS.thr_integ = 0.02;

% Roll to throttle feedforward
% PX4 Parameter: FW_T_RLL2THR
% PX4 Default: 15
Out.FW.TECS.rll2thr = 15;

climb_rate = Out.FW.TECS.climb_rate;
min_sink_rate = Out.FW.TECS.min_sink_rate;

Out.FW.TECS.STE_rate_max = max(climb_rate, eps) * SimIn.Units.g0;
Out.FW.TECS.STE_rate_min = -max(min_sink_rate, eps) * SimIn.Units.g0;


% ------------------------------------------------------------
% Fixed Wing / Path Following (L1)
% ------------------------------------------------------------

% L1 lookahead time (s)
Out.FW.L1.lookahead_time = 5;

% Minimum L1 distance for hover / very low speed operation
% Used to prevent L1 distance from collapsing to zero when V_sp is very small.
Out.FW.L1.min_hover = 20 * SimIn.Units.ft;

% Maximum commanded bank angle
Out.FW.L1.max_bank = 40 * SimIn.Units.deg;

% ------------------------------------------------------------
% Hover (Multicopter)
% ------------------------------------------------------------

% Position to velocity proportional gains in heading frame
Out.MC.Kp_pos_hdg = [0.95; 0.7; 1];

% Speed limits (upper)
Out.MC.SpdLimH = [SimIn.CtrlTrim.V_hover_enter * SimIn.Units.knot; 
                                5 * SimIn.Units.knot; 
                                8 * SimIn.Units.ft];

% Speed limits (lower)
Out.MC.SpdLimL = -[15 * SimIn.Units.knot; 
                                5 * SimIn.Units.knot; 
                                2 * SimIn.Units.ft];
end