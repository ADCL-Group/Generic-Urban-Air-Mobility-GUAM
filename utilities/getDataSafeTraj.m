% This script is used for the safe trajectory plotting, to compare the
% flown trajectory and the desired one.
close all; clc;

plotExtras = true;

SimOut = logsout{1}.Values;

plotFlightSummary(SimOut, SimIn.Units, [], SimIn.PathPlan, plotExtras);

% Save Variables to a MAT‐File
saveFlightSummary(SimOut, SimPar, SimIn, PilotInputs);