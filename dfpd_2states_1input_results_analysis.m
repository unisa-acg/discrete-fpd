% -------------------------------------------------------------------------
%
% Title:    Analysis of a D-FPD-generated control policy
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     October 2021
%
% This script analyzes the results of D-FPD applied to a probabilistic
% model foreseeing 2 states and 1 input (such as, for instance, the
% inverted pendulum). The analysis consists of (1) showing the states in
% the discrete grid for which D-FPD could optimize the joint probabilities
% (non-optimized states are usually those for which not enough information
% is available in the probabilistic model, like uniform probabilities due
% to unexplored regions of the state space); (2) verifying the convergence
% of the DKL cost; (3) checking similarity of joint PDFs (this might not be
% possible for large horizons, which cause the probability to fall below
% the floating point representativeness; (4) visualizing the control policy
% Pu for a specific state (which can be modified in the script) for each
% time instant in the optimization process. The script needs the
% 'dfpd_results_file' be defined. Some data file examples are provided in
% the 'results' folder or they can be generated through the
% 'demo_dfpd_2states_1input.m' script.
%
% -------------------------------------------------------------------------

close all;

if exist('dfpd_results_file', 'var') == 0
    error("'dfpd_results_file' is undefined in the workspace. Please define 'dfpd_results_file' to match the name of the data file containing the results of D-FPD executed on a system with 2 states and 1 input");
end

load(dfpd_results_file);

x1_card = length(model.x1_codebook);
x2_card = length(model.x2_codebook);
u_card = length(model.u_codebook);
t_card = size(policy, 1)+1;

% Visualize the states for which optimization is possible, because
% enough information is available in the probabilistic model, and those for
% which the optimization is ineffective because not enough 
% information is available in the data.
time_var_ctrl_policy_map = zeros(x1_card, x2_card);
for i1=1:x1_card
    for i2=1:x2_card
        d = abs(policy(t_card-1,i1,i2).Pu - policy(1,i1,i2).Pu);
        time_var_ctrl_policy_map(i1,i2) = log(max(d));
    end
end

figure(1);
imagesc(model.x2_codebook, model.x1_codebook, time_var_ctrl_policy_map);
colorbar;
xlabel('$x_2$', 'interpreter', 'latex');
ylabel('$x_1$', 'interpreter', 'latex');
xlim([model.x2_codebook(1), model.x2_codebook(end)]);
ylim([model.x1_codebook(1), model.x1_codebook(end)]);
axis square;

% Verify convergence of DKL cost
d_next = NaN * ones(x1_card, x2_card);
%d_matrix_norm = NaN * ones(1, t_card-2);
j = 1;
figure;

for k=t_card-1:-1:1
   
    d_curr = reshape([grid(k,:,:).d], x1_card, x2_card);
    imagesc(model.x2_codebook, model.x1_codebook, d_curr);
    %surf(model.x1_codebook, model.x2_codebook, d_curr', 'FaceAlpha', 0.5);
    colorbar;
    caxis([0, 150]);
    xlabel('$x_2$', 'interpreter', 'latex');
    ylabel('$x_1$', 'interpreter', 'latex')
    title(['$k = ' num2str(k) '$'], 'interpreter', 'latex');
    disp('Press a button to animate the graph');
    while(waitforbuttonpress==0) 
    end
    
    if k < t_card-1
       
        d_matrix_norm(k) = max(abs(d_next - d_curr), [], 'all');
        
    end
    
    d_next = d_curr;    
    
end

figure(3);
plot(1:t_card-2, d_matrix_norm);
xlabel('optimization epoch $k$', 'interpreter', 'latex');
ylabel('$\Vert d(k+1)-d(k) \Vert_\infty$', 'interpreter', 'latex');

% Randomly sample state and input trajectories, of which we compute the
% joint probability for the reference and the target models, applying
% the optimized control policy.

% This section of code is left here in order to be reused for other
% experiments, but, for the pendulum use case, probabilities get so low
% that, in the end, they are treated as zero and there is no way to compare
% Pn and Qn (joint probabilities) after many time instants. Figure 4 can be
% ignored for the pendulum use case.
number_of_simulations = 1;

% Color matrices for plots
C_P(:,:,1) = ones(x2_card, x1_card);
C_P(:,:,2) = zeros(x2_card, x1_card);
C_P(:,:,3) = zeros(x2_card, x1_card);
C_Q(:,:,1) = zeros(x2_card, x1_card);
C_Q(:,:,2) = zeros(x2_card, x1_card);
C_Q(:,:,3) = ones(x2_card, x1_card);

for i=1:number_of_simulations
    
    x1_traj = randi(x1_card, 1, t_card-1);
    x2_traj = randi(x2_card, 1, t_card-1);
    u_traj = randi(u_card, 1, t_card-1);
    
    Q = 1;
    P = 1;
    
    % Compute Q^(n-1) and P^(n-1)
    for k=1:(t_card-2)       
        
        Q = Q * model.ssg(x1_traj(k), x2_traj(k)).u(u_traj(k)).Qx(x1_traj(k+1), x2_traj(k+1)) * model.ssg(x1_traj(k), x2_traj(k)).Qu(u_traj(k));
        P = P * model.ssg(x1_traj(k), x2_traj(k)).u(u_traj(k)).Px(x1_traj(k+1), x2_traj(k+1)) * policy(k, x1_traj(k), x2_traj(k)).Pu(u_traj(k));
        
    end
    
    % Compute Q^(n-1) * Qu * Qx, fixing u(n-1)
    Qn = Q * model.ssg(x1_traj(t_card-1), x2_traj(t_card-1)).Qu(u_traj(t_card-1)) * model.ssg(x1_traj(t_card-1), x2_traj(t_card-1)).u(u_traj(t_card-1)).Qx;
    
    % Compute P^(n-1) * Pu * Px, fixing u(n-1)
    Pn = P * policy(t_card-1,x1_traj(t_card-1),x2_traj(t_card-1)).Pu(u_traj(t_card-1)) * model.ssg(x1_traj(t_card-1), x2_traj(t_card-1)).u(u_traj(t_card-1)).Px;
    
    % Since we can only visualize projections, we should not expect that
    % the joint PDFs are visually similar
    figure(4);
    s_P = surf(model.x1_codebook, model.x2_codebook, Pn', C_P, 'FaceAlpha', 0.5); hold on;
    s_P.EdgeColor = [1 0 0];
    s_P.LineStyle = ':';
    s_Q = surf(model.x1_codebook, model.x2_codebook, Qn', C_Q, 'FaceAlpha', 0.5);
    s_Q.EdgeColor = [0 0 1];
    s_Q.LineStyle = ':';
    xlabel('$x_1$', 'interpreter', 'latex');
    ylabel('$x_2$', 'interpreter', 'latex');
    zlim(1e-100*[-1, 1]);
    legend('P', 'Q');
    hold off;
    disp('Press a button to continue');
    while(waitforbuttonpress==0) 
    end
    
end

% Select state of which to visualize the control policy
x_index = [1, 3];

for k=t_card-1:-1:1
    figure(5);
    bar(1:length(model.u_codebook), policy(k,x_index(1),x_index(2)).Pu);
    xlabel('u');
    ylabel('$\tilde{P}_u$', 'interpreter', 'latex');
    title(['Policy for x1 = ' num2str(model.x1_codebook(x_index(1))) ', x2 = ' num2str(model.x2_codebook(x_index(2))) ', k = ' num2str(k)]); 
    disp('Press a button to animate the graph');
    while(waitforbuttonpress==0) 
    end
end




