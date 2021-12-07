% -------------------------------------------------------------------------
%
% Title:    D-FPD demo for a system with two states and one input
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     September 2021
%
% This script performs D-FPD optimization on a system with 2 states and 1
% input. The probabilistic state evolution model of the reference and
% target systems and the randomized control law of the reference system 
% must be provided in a file defining the 'model' variable, containing the 
% conditional probabilities and the discretized domains. See
% 'example/demo_noisy_pendulum_data_generation.m' as an example of a script
% that can generate such kind of data files. A data file that can be
% directly loaded in this script under the 'probabilistic_model_file' name
% can be found in the example/data folder. The script exports a 'policy'
% matrix containing the randomized control law for the target system. In
% this script, the user can configure the number of time instants across
% which the optimization problem is defined.
%
% -------------------------------------------------------------------------

close all;

if exist('probabilistic_model_file', 'var') == 0
    error("'probabilistic_model_file' is undefined in the workspace. Please define 'probabilistic_model_file' to match the name of the data file containing the probabilistic model of the reference/target system.");
end

load(probabilistic_model_file);

Nk = 10;
Nx1 = length(model.x1_codebook);
Nx2 = length(model.x2_codebook);
Nu = length(model.u_codebook);

% Flag to constrain the variance of the solution
constrain_variance = false;
sigma2_Qu = 0;  % For initialization only

% This is the vector of optimized divergencies that is fed-back at each
% iteration
d_curr = inf * ones(Nx1,Nx2);
d_next = inf * ones(Nx1,Nx2);

% Initial condition feedback for the solver
guess = ones(1, Nu)/Nu;

% Mask of probabilities. 1 for free probabilties, 0 for probabilities
% constrained to be zero.
u_mask = is_within_threshold(model.u_codebook, 0.5);

t = 0;
tic

for k=(Nk-1):-1:1    
    for i1=Nx1:-1:1
        for i2=Nx2:-1:1

            for h=Nu:-1:1
                
                Px = model.ssg(i1,i2).u(h).Px;
                Qx = model.ssg(i1,i2).u(h).Qx;
                Qu = model.ssg(i1,i2).Qu;

                grid(k,i1,i2).dx(h) = sum(Px .* log(Px./Qx), 'all');

                if k < Nk - 1
                    grid(k,i1,i2).r(h) = sum(Px .* d_next, 'all');
                end

            end     

            % Set the coefficients for optimization
            if k == Nk-1
                coefficients = grid(k,i1,i2).dx - log(Qu);
            else
                coefficients = grid(k,i1,i2).dx + grid(k,i1,i2).r - log(Qu);
            end

            % If a constrained optimization problem must be solved, compute
            % the variance of Qu
            if constrain_variance
                sigma2_Qu = compute_variance(model.u_codebook, Qu);
            end
            
            % Optimization problem
            [grid(k,i1,i2).Pu, grid(k,i1,i2).d] = compute_optimal_policy(coefficients, guess, u_mask, constrain_variance, model.u_codebook, 2*sigma2_Qu);

            % We also save this optimized divergence in a vector, to
            % have it available at the next iteration
            d_curr(i1,i2) = grid(k,i1,i2).d;

            % Initial condition feedback for the solver
            guess = grid(k,i1,i2).Pu;

        end
    end
    
    d_next = d_curr;
    
    t_prev = t;
    t = toc;
    disp(['Solved time instant ' num2str(k) ', Elapsed time: ' num2str(t) ' s', ', Time since last time instant: ' num2str(t-t_prev) ' s']);
end

% Export model and controller policy
for k=(Nk-1):-1:1    
    for i1=Nx1:-1:1
        for i2=Nx2:-1:1
            policy(k,i1,i2).Pu = grid(k,i1,i2).Pu;
        end
    end
end

save(['./results/' mfilename '_results_' datestr(datetime('now'), 'yymmddTHHMMSS')], 'model', 'policy', 'grid');


