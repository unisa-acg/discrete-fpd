% -------------------------------------------------------------------------
%
% Title:    compute_beta.m
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     June 2020
%
% This function computes beta as defined in the paper Karny (1996).
%
% Input Parameters:
%
%  discrete_domain: discrete sample of the domain onto which to compute the 
%                   integral
%
%  f:               matlab function corresponding to s (see paper)
%
%  gamma:           sampled function gamma as defined in the paper
%
% -------------------------------------------------------------------------

function beta = compute_beta(discrete_domain, f, gamma)

    f_s = f(discrete_domain);

    beta = trapz(discrete_domain, f_s .* (-log(gamma)));

end

