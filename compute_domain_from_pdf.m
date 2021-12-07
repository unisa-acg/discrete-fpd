% -------------------------------------------------------------------------
%
% Title:    compute_domain_from_pdf.m
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     June 2020
%
% This function computes the limited support boundaries of the PDF (assumed
% to be Gaussian) for a given multiple of the standard deviation.
%
% Input Parameters:
%
%  m:       mean of the Gaussian PDF
%
%  sigma:   standard deviation of the Gaussian PDF
%
%  n_sigma: width of the support, expressed as a multiplying factor of the
%           standard deviation
%
% -------------------------------------------------------------------------

function [lb, ub] = compute_domain_from_pdf(m, sigma, n_sigma)
    lb = m - n_sigma*sigma;
    ub = m + n_sigma*sigma;
end

