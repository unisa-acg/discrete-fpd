% -------------------------------------------------------------------------
%
% Title:    Demo for fully probabilistic design (FPD)
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     June 2020
%
% This script is a demo for the FPD as formulated by Karny (1996). 
% Conditional probabilities fx and gx are generated from a stochastic 
% dynamic model, while conditional probabilities gu are assumed to be state
% and time independent. The x-domain is increased stage by stage depending
% on the extension of the PDFs resulting from the application of the
% stochastic model. The expert's policy is to keep the input constant. 
% The user can configure the number of time instants.
%
% -------------------------------------------------------------------------

clear all;
close all;

%Number of time instants
Nk = 2;

% models (needed to generate the state pdfs)
A_f = 0.9820;
B_f = 0.2591;
sigma_fx = 1.6161;

% A_g = 0.9811;
% B_g = 0.2723;
% sigma_gx = 1.3275;

A_g = 0.2;
B_g = 0.4;
sigma_gx = 1.0;

% The expert control policy is assigned artificially and it is state and
% time independent (the expert tries to keep a constant input, regardless
% of the state it is in)
m_u = 2.5;
sigma_gu = 1.2;
gu = @(u)normal_distribution(u,m_u,sigma_gu);

[u_lb, u_ub] = compute_domain_from_pdf(m_u, sigma_gu, 5);

% initialize tree nodes
for k=Nk:-1:1
    tree(k).x = 0;
    tree(k).u = u_lb:0.1:u_ub;
end

for k=1:Nk

    x_min = min(tree(k).x);
    x_max = max(tree(k).x);
    
    for i=length(tree(k).x):-1:1        
        for h = length(tree(k).u):-1:1
            
            m_f = A_f*tree(k).x(i) + B_f*tree(k).u(h);
            m_g = A_g*tree(k).x(i) + B_g*tree(k).u(h);

            % State evolution pdfs are computed from the state evolution model
            tree(k).pdfs_x(i,h).fx = @(x)normal_distribution(x,m_f,sigma_fx);
            tree(k).pdfs_x(i,h).gx = @(x)normal_distribution(x,m_g,sigma_gx);
        
            % Define the integration interval to compute the DKLx
            [x_f_lb, x_f_ub] = compute_domain_from_pdf(m_f, sigma_fx, 5);
            [x_g_lb, x_g_ub] = compute_domain_from_pdf(m_g, sigma_gx, 5);
            domain = [min([x_f_lb, x_g_lb]), max([x_f_ub, x_g_ub])];

            % Compute DKL for x
            tree(k).alpha(i,h) = compute_alpha(domain, tree(k).pdfs_x(i,h).fx, tree(k).pdfs_x(i,h).gx);

            % Update boundaries
            if domain(1) < x_min
                x_min = domain(1);
            end

            if domain(end) > x_max
                x_max = domain(end);
            end
            
        end
        
        % Save the (sampled) control policy in the node
        tree(k).pdfs_u(i).gu = gu(tree(k).u);
        
    end       
    
    % Generate new states
    tree(k+1).x = x_min:0.1:x_max;
    
    % Initialize gamma
    if k == Nk
        tree(k+1).gamma = ones(1, length(tree(k+1).x));
    end
        
end

for k=Nk:-1:1
   
    for i=length(tree(k).x):-1:1
        
        for h=length(tree(k).u):-1:1
            tree(k).beta(i,h) = compute_beta(tree(k+1).x, tree(k).pdfs_x(i,h).fx, tree(k+1).gamma);
            tree(k).omega(i,h) = tree(k).alpha(i,h) + tree(k).beta(i,h);
        end
        
        % Compute gamma at k
        tree(k).gamma(i) = trapz(tree(k).u, tree(k).pdfs_u(i).gu .* exp(-tree(k).omega(i,:)));
        
        % Compute optimal policy
        tree(k).pdfs_u(i).fu = tree(k).pdfs_u(i).gu .* exp(-tree(k).omega(i,:)) ./ tree(k).gamma(i);
        
        % Save DKL for a-posteriori analysis
        tree(k).dkl(i) = compute_dkl(tree(k).u, tree(k).pdfs_u(i).fu, tree(k).pdfs_u(i).gu, tree(k).alpha(i,:));
    end
    
end

% Create sampled functions to export to file

for k=Nk:-1:1    
    for i=length(tree(k).x):-1:1
        for h=length(tree(k).u):-1:1
            
            tree(k).pdfs_x(i,h).fx = tree(k).pdfs_x(i,h).fx(tree(k+1).x);
            tree(k).pdfs_x(i,h).gx = tree(k).pdfs_x(i,h).gx(tree(k+1).x);
        
        end        
    end    
end

% Comparison between x-PDFs at the initial state
figure;
plot(tree(2).x, tree(1).pdfs_x(1).fx); hold on;
plot(tree(2).x, tree(1).pdfs_x(1).gx);
legend('fx', 'gx');

% Comparison between u-PDFs at the initial state
figure;
plot(tree(1).u, tree(1).pdfs_u(1).fu); hold on;
plot(tree(1).u, tree(1).pdfs_u(1).gu);
legend('fu', 'gu');

% Comparison between fu at different x(i)
figure;
for i=1:length(tree(2).x)   
    plot(tree(2).u, tree(2).pdfs_u(i).fu); hold on;
end

save(['./results/' mfilename '_results_' datestr(datetime('now'), 'yymmddTHHMMSS')]);