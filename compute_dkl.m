% -------------------------------------------------------------------------
%
% Title:    compute_dkl.m
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     June 2020
%
% This function computes the local Kullback-Leibler divergence between two 
% PDFs, each given by the product of two conditional PDFs defined on the
% input and on the state domains.
%
% Input Parameters:
%
%  domain:  sampled input domain onto which to compute the divergence
%
%  fu:      local conditional PDF on the input variable, component of the 
%           left-hand term of the Kullback-Leibler divegence
%
%  gu:      local conditional PDF on the input variable, component of the 
%           right-hand term of the Kullback-Leibler divegence
%
%  alpha:   Kullback-Leibler divegence between PDFs defined on the state
%           variable
%
% -------------------------------------------------------------------------

function dkl = compute_dkl(domain, fu, gu, alpha)

    dkl_u = trapz(domain, fu .* (log(fu) - log(gu)));
    
    e_alpha = trapz(domain, fu .* alpha);
    
    dkl = dkl_u + e_alpha;

end

