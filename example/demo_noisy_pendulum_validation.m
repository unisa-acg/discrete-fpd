% -------------------------------------------------------------------------
%
% Title:    Validation of data-driven control policy for the pendulum
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     October 2021
%
% This script loads a data-driven control policy and validates it through
% simulation. The control policy data file should be provided through the
% variable 'control_policy_file'. The same file must contain the
% discretized model of the system. An example is provided in the ../results
% folder or more policies can be generated through the
% 'demo_dfpd_2states_1input.m' script.
%
% -------------------------------------------------------------------------

close all;

if exist('control_policy_file', 'var') == 0
    error("'control_policy_file' is undefined in the workspace. Please define 'control_policy_file' to match the name of the data file containing the results of D-FPD");
end

if exist('model_based_simulations_file', 'var') == 0
    error("'model_based_simulations_file' is undefined in the workspace. Please define 'model_based_simulations_file' to match the name of the data file containing the model based simulations for the target pendulum");
end

% Load discretized model and data-driven control policy
load(control_policy_file);

model_based_control = 0;
data_driven_controller_block = [pendulum_model_name '/data_driven_controller_block'];
reference_block = [pendulum_model_name '/Reference'];

% Setup Simulink simulation for target system
load_system('target_pendulum_setup');

set_param(data_driven_controller_block,'commented','off');
set_param(reference_block,'commented','on');

% Execute simulation with target system
simulation_target = sim('target_pendulum_setup');

% Setup Simulink simulation for reference system
load_system('reference_pendulum_setup');

set_param(data_driven_controller_block,'commented','off');
set_param(reference_block,'commented','on');

% Execute simulation with reference system
simulation_reference = sim('reference_pendulum_setup');

% Show trajectory
figure;
plot(simulation_target.q);
xlabel('Time (s)');
ylabel('Position (rad)');
hold on;
plot(simulation_reference.q);

figure;
plot(simulation_target.qd);
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
hold on;
plot(simulation_reference.qd);

figure;
plot(simulation_target.tau);
xlabel('Time (s)');
ylabel('Torque (Nm)');
hold on;
plot(simulation_reference.tau);

% Load model-based simulations and plot data for comparison
sim_target = load(model_based_simulations_file);
number_of_simulations = length(sim_target.simulations);

for i=1:number_of_simulations
    figure(4); 
    plot(sim_target.simulations(i).q, 'c', 'linewidth', 0.5); hold on;

    figure(5);
    plot(sim_target.simulations(i).qd, 'c', 'linewidth', 0.5); hold on;
end

figure(4);
plot(simulation_target.q, 'r', 'linewidth', 2);
xlabel('Time (s)');
ylabel('Position (rad)');

figure(5);
plot(simulation_target.qd, 'r', 'linewidth', 2);
xlabel('Time (s)');
ylabel('Velocity (rad/s)');

% Save results of simulation for further analyses
save(['./data/pendulum_dfpd_validation_' datestr(datetime('now'), 'yymmddTHHMMSS')], 'simulation_target');

