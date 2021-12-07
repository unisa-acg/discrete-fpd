% -------------------------------------------------------------------------
%
% Title:    normal_distribution.m
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     June 2020
%
% This function computes a symbolic normal distribution, given a symbolic
% random variable, mean and standard deviation.
%
% Input Parameters:
%
%  x:       symbolic random variable
%
%  m:       mean of normal distribution
%
%  sigma:   standard deviation of normal distribution
%
% -------------------------------------------------------------------------

function pdf = normal_distribution(x, m, sigma)

    pdf = 1/sigma/sqrt(2*pi)*exp(-1/2*((x-m)/sigma).^2);

end

