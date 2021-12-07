% -------------------------------------------------------------------------
%
% Title:    compute_optimal_policy.m
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     June 2020
%
% This function performs the local optimization of the discrete fully
% probabilistic design using fmincon.
%
% Input Parameters:
%
%  coefficients:        coefficients of the optimization (see D-FPD)
%
%  guess:               initial guess for a solution
%
%  u_mask:              binary vector of the same size as u_vector. 0
%                       stands for masked probability (constrained to be
%                       zero), 1 stands for unconstrained probability.
%
%  constrain_variance:  flag indicating whether a variance constraint has
%                       to be set in the optimization problem
%
%  u_vector:            discrete domain of the input (used to compute the
%                       variance when the variance constraint is imposed)
%
%  sigma2_con:          desired variance of the output probabilites
%
% Output Parameters:
%
%  Pu:              conditional probabilities corresponding to the optimal
%                   cost
%
%  d:               optimal cost, i.e. Kullback-Leibler divergence between
%                   probabilities given by the product of the conditional 
%                   probabilities defined on the state and those defined on 
%                   the input
% -------------------------------------------------------------------------

function [Pu, d] = compute_optimal_policy(coefficients, guess, u_mask, constrain_variance, u_vector, sigma2_con)

    m = length(coefficients);
    m_free = sum(u_mask);

    options = optimoptions(@fmincon,'Display','off','Algorithm','interior-point');
    
    coefficients_free = coefficients(u_mask);
    guess_free = guess(u_mask);
    
    % Un-comment the following line to de-activate initial condition
    % feedback
    % guess = ones(1,m)/m;
    
    % The variance constraint should be refactored to account for null
    % probabilities given through u_mask
    constraints_function = [];
    
    if constrain_variance        
        constraints_function = @(p)non_linear_constraints(p, u_vector, sigma2_con);
    end
    
    [Pu_free, d] = fmincon(@(p)objective_function(p,coefficients_free),guess_free,[],[],ones(1,m_free),1,zeros(1,m_free),ones(1,m_free),constraints_function,options);
    
    Pu = zeros(1,m);
    Pu(u_mask) = Pu_free;
    
end

