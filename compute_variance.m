% -------------------------------------------------------------------------
%
% Title:    compute_variance.m
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     July 2020
%
% This function computes the discrete variance, given a set of elements
% with their proababilities.
%
% Input Parameters:
%
%  discrete_domain: possible values of the discrete random variable
%
%  probabilities:   probabilities of the discrete values
%
% -------------------------------------------------------------------------

function [sigma2] = compute_variance(discrete_domain, probabilities)

    discrete_domain_sq = discrete_domain.^2;
    
    sigma2 = sum(probabilities .* discrete_domain_sq) - sum(probabilities .* discrete_domain)^2;
    
end

