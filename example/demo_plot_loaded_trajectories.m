% -------------------------------------------------------------------------
%
% Title:    Plot trajectories from Simulink simulations
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     September 2021
%
% This scripts plots trajectories from a set of Simulink simulations, by
% loading them from one data file called 'simulations_data_file', that must
% exist in the worspace before this script is executed. One way of
% generating such trajectories is through the
% demo_noisy_pendulum_data_generation script.
%
% -------------------------------------------------------------------------

if exist('simulations_data_file', 'var') == 0
    error("'simulations_data_file' is undefined in the workspace.");
end

loaded_file = load(simulations_data_file);

plot_trajectories(loaded_file.simulations);