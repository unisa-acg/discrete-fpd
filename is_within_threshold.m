% -------------------------------------------------------------------------
%
% Title:    is_within_threshold.m
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     November 2021
%
% This function returns a binary vector with as many elements as the input
% vector. For each element, if -x_max <= x <= x_max, the output is equal
% to 1 (input not masked), otherwise it is equal to zero (input masked).
%
% Input Parameters:
%
%  x:       input vector
%
%  x_max:   threshold
%
% -------------------------------------------------------------------------

function [x_mask] = is_within_threshold(x, x_max)

    x_mask_lower = (x >= -x_max);
    x_mask_upper = (x <= x_max);
    
    x_mask = x_mask_lower & x_mask_upper;

end

