% -------------------------------------------------------------------------
%
% Title:    non_linear_constraints.m
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     July 2020
%
% Non-linear constraints of the D-FPD optimization problem to be used by
% fmincon. At the moment, the only constraint is on the variance of the
% output probabilities.
%
% Input Parameters:
%
%  p:           vector of variables to be constrained
%
%  u_vector:    sampled input domain
%
%  sigma2_con:  desired variance
%
% -------------------------------------------------------------------------

function [c, ceq] = non_linear_constraints(p, u_vector, sigma2_con)

    % There are no inequality constraint
    c = [];
    
    % Variance constraint
    
    u_vector_sq = u_vector.^2;
    
    ceq = sum(p .* u_vector_sq) - sum(p .* u_vector)^2 - sigma2_con;
    
end

