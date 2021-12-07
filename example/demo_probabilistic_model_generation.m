% -------------------------------------------------------------------------
%
% Title:    Probabilistic model generation demo from trajectory data
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     September 2021
%
% This scripts loads reference and target pendulums trajectories,
% pre-process them and computes conditional probabilities of the state
% evolution model (Px and Qx) and conditional probabilities of the
% randomized control law (Qu). Pre-processing consists of defining the
% state and input domains boundaries, discretization (or quantization) and
% down-sampling. By assuming time-invariance, down-sampling is not usually
% needed and is commented out in this script. The scripts also verifies
% that conditional probabilities sum to one and exports both the
% probabilistic model and discrete domain codebooks to a MATLAB data file.
%
% -------------------------------------------------------------------------

close all;

sim_reference = load(reference_simulation_output);
sim_target = load(target_simulation_output);

sim_reference = sim_reference.simulations;
sim_target = sim_target.simulations;

number_of_simulations_ref = length(sim_reference);
number_of_simulations_tar = length(sim_target);

q_min = +inf;
q_max = -inf;
qd_min = +inf;
qd_max = -inf;
tau_ref_max = -inf;
tau_tar_max = -inf;
t_min = +inf;
t_max = -inf;

% We use fixed input boundaries and then map different input domains to
% such boundaries
u_min = -1;
u_max = 1;

% We get maximum and minimum values of all quantities and time to proceed
% to re-sampling and quantization/discretization
  
for i=1:(number_of_simulations_ref + number_of_simulations_tar)   % Here we loop across all simulations, for each pendulum

    if i <= number_of_simulations_ref
        trajectory = sim_reference(i);
        tau_ref_max = max([tau_ref_max; abs(trajectory.tau.Data)]);             
    else
        trajectory = sim_target(i-number_of_simulations_ref);
        tau_tar_max = max([tau_tar_max; abs(trajectory.tau.Data)]);
    end

    q_min = min([trajectory.q.Data; q_min]);
    q_max = max([trajectory.q.Data; q_max]);
    qd_min = min([trajectory.qd.Data; qd_min]);
    qd_max = max([trajectory.qd.Data; qd_max]);
    t_min = min([trajectory.tout; t_min]);
    t_max = max([trajectory.tout; t_max]);

end


% Quantization

q_codebook = linspace(q_min, q_max, 30);
qd_codebook = linspace(qd_min, qd_max, 30);
u_codebook = linspace(u_min, u_max, 40);

delta_q = q_codebook(2)-q_codebook(1);
delta_qd = qd_codebook(2)-qd_codebook(1);
delta_u = u_codebook(2)-u_codebook(1);

% Add tolerance to prevent numerical errors
epsilon = 1e-8;

q_partition = (q_min+delta_q/2):delta_q:(q_max-delta_q/2+epsilon);
qd_partition = (qd_min+delta_qd/2):delta_qd:(qd_max-delta_qd/2+epsilon);
u_partition = (u_min+delta_u/2):delta_u:(u_max-delta_u/2+epsilon);

for i=number_of_simulations_ref:-1:1
    
    reference_trajectories(i).time = sim_reference(i).tout;

    [indices, quants] = quantiz(sim_reference(i).q.Data, q_partition, q_codebook);
    reference_trajectories(i).q.Data = quants;
    reference_trajectories(i).q.Indices = indices + ones(length(indices),1);
    
    [indices, quants] = quantiz(sim_reference(i).qd.Data, qd_partition, qd_codebook);
    reference_trajectories(i).qd.Data = quants;
    reference_trajectories(i).qd.Indices = indices + ones(length(indices),1);
    
    [indices, quants] = quantiz(sim_reference(i).tau.Data ./ tau_ref_max, u_partition, u_codebook);
    reference_trajectories(i).u.Data = quants;
    reference_trajectories(i).u.Indices = indices + ones(length(indices),1);
    
%     figure(1);
%     plot(reference_trajectories(i).time, reference_trajectories(i).q.Indices); hold on;
%     
%     figure(2);
%     plot(reference_trajectories(i).time, reference_trajectories(i).qd.Indices); hold on;

end

for i=number_of_simulations_tar:-1:1
    
    target_trajectories(i).time = sim_target(i).tout;

    [indices, quants] = quantiz(sim_target(i).q.Data, q_partition, q_codebook);
    target_trajectories(i).q.Data = quants;
    target_trajectories(i).q.Indices = indices + ones(length(indices),1);
    
    [indices, quants] = quantiz(sim_target(i).qd.Data, qd_partition, qd_codebook);
    target_trajectories(i).qd.Data = quants;
    target_trajectories(i).qd.Indices = indices + ones(length(indices),1);
    
    [indices, quants] = quantiz(sim_target(i).tau.Data ./ tau_tar_max, u_partition, u_codebook);
    target_trajectories(i).u.Data = quants;
    target_trajectories(i).u.Indices = indices + ones(length(indices),1);
    
%     figure(3);
%     plot(target_trajectories(i).time, target_trajectories(i).q.Indices); hold on;
%     
%     figure(4);
%     plot(target_trajectories(i).time, target_trajectories(i).qd.Indices); hold on;

end

% Downsampling
%
% time = linspace(t_min, t_max, 100)';
% 
% for i=number_of_simulations:-1:1
% 
%     figure;
%     plot(sim_reference(i).tau); hold on;
%     
%     sim_reference(i).q = timeseries(interp1(sim_reference(i).q.Time, sim_reference(i).q.Data, time), time);
%     sim_reference(i).qd = timeseries(interp1(sim_reference(i).qd.Time, sim_reference(i).qd.Data, time), time);
%     sim_reference(i).tau = timeseries(interp1(sim_reference(i).tau.Time, sim_reference(i).tau.Data, time), time);
%     sim_reference(i).tout = time;
% 
%     plot(sim_reference(i).tau);
%     
% end

% Computation of probabilities

q_card = length(q_codebook);
qd_card = length(qd_codebook);
u_card = length(u_codebook);

% The following conditions generate NaN probabilities:
%   - a state is never visited, i.e. times_visited == 0 (Qu is NaN)
%   - an input is never selected in a state, i.e. count_input(k) == 0 (Px
%   and Qx are NaN)
% The following conditions generate 0 probabilities (this is an issue if
% the D-FPD solver uses the DKL):
%   - a state is never reached from a given state when applying a given
%   input (Px and Qx are zero)
% We define the following thresholds (in the domain of counts) to cope with
% the conditions above
times_visited_threshold = 1/(q_card*qd_card);
count_input_threshold = times_visited_threshold/u_card;
count_next_state_threshold = count_input_threshold/(q_card*qd_card);

% Initialize state space grids for target and reference
for i=q_card:-1:1
    for j=qd_card:-1:1
        for k=u_card:-1:1
            % We store the number of times we get in a state when we start
            % form a given state and use a given input
            ssgr(i,j).input(k).count_next_state = count_next_state_threshold * ones(q_card, qd_card);
            ssgt(i,j).input(k).count_next_state = count_next_state_threshold * ones(q_card, qd_card);
        end
        
        % We store the number of times we select an input for a given state
        ssgr(i,j).count_input = count_input_threshold * ones(1, u_card);
        ssgt(i,j).count_input = count_input_threshold * ones(1, u_card);
        
        % We store the number of times we visit a state
        ssgr(i,j).times_visited = times_visited_threshold;
        ssgt(i,j).times_visited = times_visited_threshold;
    end
end

% Compute the counting functions initialized above
for i=1:number_of_simulations_ref
    
    trajectory = reference_trajectories(i);
    
    for j=1:(length(trajectory.time)-1)
             
        ssgr(trajectory.q.Indices(j), trajectory.qd.Indices(j)).input(trajectory.u.Indices(j)).count_next_state(trajectory.q.Indices(j+1), trajectory.qd.Indices(j+1)) = ...
            ssgr(trajectory.q.Indices(j), trajectory.qd.Indices(j)).input(trajectory.u.Indices(j)).count_next_state(trajectory.q.Indices(j+1), trajectory.qd.Indices(j+1)) + 1;
            
        ssgr(trajectory.q.Indices(j), trajectory.qd.Indices(j)).times_visited = ssgr(trajectory.q.Indices(j), trajectory.qd.Indices(j)).times_visited + 1;

        ssgr(trajectory.q.Indices(j), trajectory.qd.Indices(j)).count_input(trajectory.u.Indices(j)) = ...
            ssgr(trajectory.q.Indices(j), trajectory.qd.Indices(j)).count_input(trajectory.u.Indices(j)) + 1;
        
    end
end

for i=1:number_of_simulations_tar
    
    trajectory = target_trajectories(i);
    
    for j=1:(length(trajectory.time)-1)
        
        ssgt(trajectory.q.Indices(j), trajectory.qd.Indices(j)).input(trajectory.u.Indices(j)).count_next_state(trajectory.q.Indices(j+1), trajectory.qd.Indices(j+1)) = ...
            ssgt(trajectory.q.Indices(j), trajectory.qd.Indices(j)).input(trajectory.u.Indices(j)).count_next_state(trajectory.q.Indices(j+1), trajectory.qd.Indices(j+1)) + 1;
            
        ssgt(trajectory.q.Indices(j), trajectory.qd.Indices(j)).times_visited = ssgt(trajectory.q.Indices(j), trajectory.qd.Indices(j)).times_visited + 1;
        
        ssgt(trajectory.q.Indices(j), trajectory.qd.Indices(j)).count_input(trajectory.u.Indices(j)) = ...
            ssgt(trajectory.q.Indices(j), trajectory.qd.Indices(j)).count_input(trajectory.u.Indices(j)) + 1;        
        
    end
end

% Count total number of times a state is reached and compute conditional
% probabilities
for i=q_card:-1:1
    for j=qd_card:-1:1
        
        % Saturation to avoid times_visited = 0
        ssgr(i,j).times_visited = max([times_visited_threshold, ssgr(i,j).times_visited]);
        ssgt(i,j).times_visited = max([times_visited_threshold, ssgt(i,j).times_visited]);
        
        for k=u_card:-1:1
            
            % Saturation to avoid count_next_state = 0
            ssgr(i,j).input(k).count_next_state(ssgr(i,j).input(k).count_next_state == 0) = count_next_state_threshold;
            
            ssgr(i,j).input(k).Qx = ssgr(i,j).input(k).count_next_state ./ ssgr(i,j).count_input(k);
            ssgt(i,j).input(k).Px = ssgt(i,j).input(k).count_next_state ./ ssgt(i,j).count_input(k);
            
            ssgr(i,j).input(k).Qx(isnan(ssgr(i,j).input(k).Qx)) = 0;
            ssgt(i,j).input(k).Px(isnan(ssgt(i,j).input(k).Px)) = 0;
        end
        
        ssgr(i,j).Qu = ssgr(i,j).count_input ./ ssgr(i,j).times_visited;
        ssgr(i,j).Qu(isnan(ssgr(i,j).Qu)) = 0;
        
        visited_states_reference(i,j) = ssgr(i,j).times_visited;
        visited_states_target(i,j) = ssgt(i,j).times_visited;        
    end
end

figure;
surf(qd_codebook, q_codebook, visited_states_reference);
figure;
surf(qd_codebook, q_codebook, visited_states_target);

% Verify probabilistic model
for i=q_card:-1:1
    for j=qd_card:-1:1
        for k=u_card:-1:1
            
            p_sum = sum(ssgr(i,j).input(k).Qx, 'all');
            
            if ssgr(i,j).count_input(k) ~= 0 && abs(p_sum - 1) > 1e-5
                sum(ssgr(i,j).input(k).count_next_state, 'all')
                ssgr(i,j).count_input(k)
                error('Conditional probabilities Qx do not sum to 1');
            end
            
            p_sum = sum(ssgt(i,j).input(k).Px, 'all');
            
            if ssgt(i,j).count_input(k) ~= 0 && abs(p_sum - 1) > 1e-5
                error('Conditional probabilities Px do not sum to 1');
            end
            
        end       
        
        p_sum = sum(ssgr(i,j).Qu);
        
        if (ssgr(i,j).times_visited ~= 0) && abs(p_sum - 1) > 1e-5
            error('Conditional probabilities Qu do not sum to 1');
        end
        
    end
end

% Export probabilistic model

for i=q_card:-1:1
    for j=qd_card:-1:1
        for k=u_card:-1:1            
            model.ssg(i,j).u(k).Qx = ssgr(i,j).input(k).Qx;
            model.ssg(i,j).u(k).Px = ssgt(i,j).input(k).Px;
            model.ssg(i,j).Qu = ssgr(i,j).Qu;            
        end
    end
end

model.x1_codebook = q_codebook;
model.x2_codebook = qd_codebook;
model.u_codebook = u_codebook;
model.tau_ref_max = tau_ref_max;
model.tau_tar_max = tau_tar_max;

save(['./data/pendulum_probabilistic_model_' datestr(datetime('now'), 'yymmddTHHMMSS')], 'model');