% -------------------------------------------------------------------------
%
% Title:    Data generation demo for an actuated noisy pendulum
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     July 2020
%
% This scripts generates random position references that allow an actuated 
% noisy pendulum to move from the stable equilibrium state (-pi/2) to the 
% unstable equilibrium state (pi/2) and executes them on the simulated 
% system. 30% of reference trajectories have one velocity switching point
% (i.e. one acceleration and one deceleration segment), 70% of trajectories
% have three switching points (i.e. 2 acceleration segments and 2
% deceleration segments). The torques generated by the model-based
% controller and the measured velocities and positions are collected and
% plotted together for all the simulations.
%
% -------------------------------------------------------------------------

close all;

if exist('pendulum_model_name', 'var') == 0
    error("'pendulum_model_name' is undefined in the workspace. Please define 'pendulum_model_name' to match the name of the pendulum's Simulink model.");
end

load_system(pendulum_model_name);

model_based_control = 1;

data_driven_controller_block = [pendulum_model_name '/data_driven_controller_block'];
reference_block = [pendulum_model_name '/Reference'];
set_param(data_driven_controller_block,'commented','on');
set_param(reference_block,'commented','off');

min_pseudovelocity = 0.5;
pseudovelocity_range = 4.5;
delta_lambda = 0.01;
variance = 1;

n_simulations = 100;
n_simulations_1sp = ceil(0.3 * n_simulations);
sim_int_index = n_simulations - n_simulations_1sp;

for i=n_simulations:-1:(sim_int_index+1)
    
    disp(['Executing simulation ' num2str((n_simulations - i + 1)) '/' num2str(n_simulations)]);
    
    [r, rd] = generate_random_1sp_trajectory(min_pseudovelocity, pseudovelocity_range, delta_lambda);
    
    simulations(i) = sim(pendulum_model_name);
    
end

for i=sim_int_index:-1:1
    
    disp(['Executing simulation ' num2str((n_simulations - i + 1)) '/' num2str(n_simulations)]);
    
    [r, rd] = generate_random_3sp_trajectory(min_pseudovelocity, pseudovelocity_range, delta_lambda, variance);

    simulations(i) = sim(pendulum_model_name);
    
end

plot_trajectories(simulations);

save(['./data/' pendulum_model_name '_' datestr(datetime('now'), 'yymmddTHHMMSS')], 'simulations');

