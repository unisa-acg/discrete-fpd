% -------------------------------------------------------------------------
%
% Title:    objective_function.m
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     June 2020
%
% Objective function of the D-FPD optimization problems to be used by
% fmincon.
%
% Input Parameters:
%
%  p: vector of variables to be optimized
%
%  c: vector of coefficients as defined by D-FPD
%
% -------------------------------------------------------------------------

function f = objective_function(p, c)
    
    f = sum(p.*log(p) + p.*c);

end

