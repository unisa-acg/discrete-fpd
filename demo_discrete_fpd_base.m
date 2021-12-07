% -------------------------------------------------------------------------
%
% Title:    Demo for discrete fully probabilistic design (D-FPD)
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     June 2020
%
% This script is a demo for the D-FPD. Conditional probabilities Px and Qx
% are generated from a stochastic dynamic model, while conditional
% probabilities Qu are assumed to be state and time independent.
% Probabilities Px, Qx and Qu are normalized when close to the state's and
% input's domain boundaries. The expert's policy is to keep the input 
% constant. The user can configure the number of time instants, the state
% domain and the number of samples therein, the input domain and the number
% of samples therein, the actual and expert's state evolution model, the 
% value of the input that the expert keeps constant, as well as whether to
% include a constraint on the variance of the resulting optimal policy. At
% the end of the script, a graph is generated to compare the joint
% probabilities P and Q on a projected space.
%
% -------------------------------------------------------------------------

clear all;
close all;

% Definition of the discrete state space
x_lb = -15;
x_ub = 18;
delta_x = 0.1;

x_vector = x_lb:delta_x:x_ub;

% Definition of the discrete input space
u_lb = -4;
u_ub = 9;
delta_u = 1;

u_vector = u_lb:delta_u:u_ub;

Nk = 3;
Nx = length(x_vector);
Nu = length(u_vector);

% models (needed to generate the state probabilities)
A_P = 0.9820;
B_P = 0.2591;
sigma_Px = 1.6161;

% A_Q = 0.9811;
% B_Q = 0.2723;
% sigma_Qx = 1.3275;

A_Q = 0.2;
B_Q = 0.4;
sigma_Qx = 1.0;

% The expert control policy is assigned artificially and it is state and
% time independent (the expert tries to keep a constant input, regardless
% of the state it is in)
sigma_Qu = 1.2;
Qu = compute_probabilities(u_vector, 2.5, sigma_Qu);

% Flag to constrain the variance of the solution
constrain_variance = true;

% This is the vector of optimized divergencies that is fed-back at each
% iteration
d_curr = inf * ones(1,Nx);
d_next = inf * ones(1,Nx);

% Initial condition feedback for the solver
guess = ones(1, Nu)/Nu;

tic

for k=Nk:-1:1
    for i=Nx:-1:1
       
        grid(k,i).x = x_vector(i);
        x_i = x_vector(i);
        
        if k < Nk           
            for h=Nu:-1:1
                
                u_h = u_vector(h);

                m_P = A_P*x_i + B_P*u_h;
                m_Q = A_Q*x_i + B_Q*u_h;

                % State evolution probabilities are computed from the state
                % evolution model
                Px = compute_probabilities(x_vector, m_P, sigma_Px);
                Qx = compute_probabilities(x_vector, m_Q, sigma_Qx);
                
                for j=Nx:-1:1
                        
                    grid(k+1,j).Px(i,h) = Px(j);
                    grid(k+1,j).Qx(i,h) = Qx(j);
                    
                end
                
                grid(k,i).dx(h) = sum(Px .* log(Px./Qx));
                grid(k,i).Qu(h) = Qu(h);
                
                if k < Nk - 1
                    grid(k,i).r(h) = sum(Px .* d_next);
                end

            end     
            
            % Set the coefficients for optimization
            if k == Nk-1
                coefficients = grid(k,i).dx - log(grid(k,i).Qu);
            else
                coefficients = grid(k,i).dx + grid(k,i).r - log(grid(k,i).Qu);
            end
        
            % If a constrained optimization problem must be solved, compute
            % the variance of Qu
            sigma2_Qu = compute_variance(u_vector, grid(k,i).Qu);
            
            % Optimization problem
            [grid(k,i).Pu, grid(k,i).d] = compute_optimal_policy(coefficients, guess, constrain_variance, u_vector, 2*sigma2_Qu);
            
            % We also save this optimized divergence in a vector, to
            % have it available at the next iteration
            d_curr(i) = grid(k,i).d;
            
            % Initial condition feedback for the solver
            guess = grid(k,i).Pu;
            
        end        
    end
    
    d_next = d_curr;
end

toc

% In order to compare joint probabilities P and Q (that are
% four-dimensional functions of u0, u1, x1 and x2, for two time instants),
% we visualize projections in the space u1-x2. Thus we assume that u0 and
% x1 are given. x0 is also given and is equal to 0.

u0 = 2;
x1 = B_P * u0;  % take the mean (x0 = 0)

x0_index = find(x_vector == 0, 1);
x1_index = find(abs(x_vector-x1) < delta_x/2, 1);
u0_index = find(abs(u_vector-u0) < 1e-3, 1);

% P(x1|x0,u0)*P(u0|x0) and Q(x1|x0,u0)*P(u0|x0)
P1 = grid(2,x1_index).Px(x0_index,u0_index) * grid(1,x0_index).Pu(u0_index);
Q1 = grid(2,x1_index).Qx(x0_index,u0_index) * grid(1,x0_index).Qu(u0_index);

% P(u1|x1) and Q(u1|x1)
Pu1 = grid(2,x1_index).Pu;
Qu1 = grid(2,x1_index).Qu;

for h = Nu:-1:1    
    for i = Nx:-1:1
        
        % P(x2|x1,u1) and Q(x2|x1,u1)
        Px2 = grid(3,i).Px(x1_index,h);
        Qx2 = grid(3,i).Qx(x1_index,h);
        
        % P2 = Px2 * Pu1 and Q2 = Qx2 * Qu1
        P2 = Px2 * Pu1(h);
        Q2 = Qx2 * Qu1(h);
        
        % Joint probabilities P and Q
        P(i,h) = P2 * P1;
        Q(i,h) = Q2 * Q1;
        
    end
end

% Plots

% Color matrices
C_P(:,:,1) = ones(Nu, Nx);
C_P(:,:,2) = zeros(Nu, Nx);
C_P(:,:,3) = zeros(Nu, Nx);
C_Q(:,:,1) = zeros(Nu, Nx);
C_Q(:,:,2) = zeros(Nu, Nx);
C_Q(:,:,3) = ones(Nu, Nx);

figure;
s_P = surf(x_vector, u_vector, P', C_P, 'FaceAlpha', 0.5); hold on;
s_P.EdgeColor = [1 0 0];
s_P.LineStyle = ':';
s_Q = surf(x_vector, u_vector, Q', C_Q, 'FaceAlpha', 0.5);
s_Q.EdgeColor = [0 0 1];
s_Q.LineStyle = ':';
xlabel('$x_2$', 'interpreter', 'latex');
ylabel('$u_1$', 'interpreter', 'latex');
legend('P', 'Q');

save(['./results/' mfilename '_results_' datestr(datetime('now'), 'yymmddTHHMMSS')]);


