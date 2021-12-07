% -------------------------------------------------------------------------
%
% Title:    compute_alpha.m
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     June 2020
%
% This function computes alpha as defined in the paper Karny (1996), which
% is the Kullback-Leibler divergence between two conditional pdfs defined
% on the state domain. Inputs are matlab functions, the result is a scalar.
%
% Input Parameters:
%
%  domain:  it is a 2-dimensional vector containing the domain limits of 
%           the state variable
%
%  f:       matlab function representing the left-hand term of the 
%           Kullback-Leibler divergence
%
%  g:       matlab function representing the right-hand term of the
%           Kullback-Leibler divergence
%
% -------------------------------------------------------------------------

function alpha = compute_alpha(domain, f, g)

    integrand = @(x) f(x) .* (log(f(x)) - log(g(x)));
    
    alpha = integral(integrand, domain(1), domain(end));

end

