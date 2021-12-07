% -------------------------------------------------------------------------
%
% Title:    Demo to compare results from FPD and D-FPD
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     June 2020
%
% This script compares results from FPD and D-FPD in order to validate the
% latter. The results to be analyzed must be contained in mat files on
% disc.
%
% -------------------------------------------------------------------------

close all;
clear all;

% Load continuous and discrete solutions
continuous = load('results/demo_continuous_fpd_results_200623T140000.mat');
discrete = load('results/demo_discrete_fpd_results_200623T143028.mat');

% Define an initial index at which to compare the results of the
% optimization
k_index = 2;
d_x_index = 211;
c_x_index = 151;
d_h_index = 7;
c_h_index = 56;

% Compare pdf with histogram for generic fx and Px

for i=length(discrete.x_vector):-1:1
    Px(i) = discrete.grid(k_index+1,i).Px(d_x_index,d_h_index);
end

figure;
bar(discrete.x_vector, Px); hold on;
plot(continuous.tree(k_index+1).x, continuous.tree(k_index).pdfs_x(c_x_index,c_h_index).fx);
title('Comparison between Px and fx');

% Compare alpha and dx for all x_{k-1} and u_{k-1}, k fixed

dx_map = ones(length(discrete.x_vector), length(discrete.u_vector));

for i=1:length(discrete.x_vector)
    dx_map(i,:) = discrete.grid(k_index, i).dx;
end

C_d(:,:,1) = ones(length(discrete.u_vector), length(discrete.x_vector));
C_d(:,:,2) = zeros(length(discrete.u_vector), length(discrete.x_vector));
C_d(:,:,3) = zeros(length(discrete.u_vector), length(discrete.x_vector));

C_c(:,:,1) = zeros(length(continuous.tree(k_index).u), length(continuous.tree(k_index).x));
C_c(:,:,2) = zeros(length(continuous.tree(k_index).u), length(continuous.tree(k_index).x));
C_c(:,:,3) = ones(length(continuous.tree(k_index).u), length(continuous.tree(k_index).x));

figure;
s_d = surf(discrete.x_vector, discrete.u_vector, dx_map', C_d, 'FaceAlpha', 0.5); hold on;
s_d.EdgeColor = 'none';
s_c = surf(continuous.tree(k_index).x, continuous.tree(k_index).u, continuous.tree(k_index).alpha', C_c, 'FaceAlpha', 0.5);
s_c.EdgeColor = 'none';
xlabel('$x_{k-1}$', 'interpreter', 'latex');
ylabel('$u_{k-1}$', 'interpreter', 'latex');
title('Comparison between dx and alpha');

% Compare alpha and dx for fixed k and x_{k-1}

figure;
plot(discrete.u_vector, discrete.grid(k_index, d_x_index).dx, 'o'); hold on;
plot(continuous.tree(k_index).u, continuous.tree(k_index).alpha(c_x_index,:));
title('Comparison between dx and alpha');

% Compare gu and Qu

figure;
bar(discrete.u_vector, discrete.grid(k_index,d_x_index).Qu); hold on;
plot(continuous.tree(k_index).u, continuous.tree(k_index).pdfs_u(c_x_index).gu);
title('Comparison between Qu and gu');

% Compare optimal policies at k = 2, x given by indices above

figure;
bar(discrete.u_vector, discrete.grid(k_index,d_x_index).Pu); hold on;
plot(continuous.tree(k_index).u, continuous.tree(k_index).pdfs_u(c_x_index).fu);
title('Comparison between Pu and fu');

% Compare optimal local costs (local DKL)

for i=length(discrete.x_vector):-1:1
    d_dkl(i) = discrete.grid(k_index,i).d;
end

figure;
plot(discrete.x_vector, d_dkl, 'o'); hold on;
plot(continuous.tree(k_index).x, continuous.tree(k_index).dkl);
title('Comparison between discrete and continuous local DKL');

% Change indices

k_index = 1;
d_x_index = 151;
c_x_index = 1;

% Compare alpha and dx for fixed k and x_{k-1}

figure;
plot(discrete.u_vector, discrete.grid(k_index, d_x_index).dx, 'o'); hold on;
plot(continuous.tree(k_index).u, continuous.tree(k_index).alpha(c_x_index,:));
title('Comparison between dx and alpha');

% Compare gu and Qu

figure;
bar(discrete.u_vector, discrete.grid(k_index,d_x_index).Qu); hold on;
plot(continuous.tree(k_index).u, continuous.tree(k_index).pdfs_u(c_x_index).gu);
title('Comparison between Qu and gu');

% Compare optimal policies at k = 1, x given by indices above

figure;
bar(discrete.u_vector, discrete.grid(k_index,d_x_index).Pu); hold on;
plot(continuous.tree(k_index).u, continuous.tree(k_index).pdfs_u(c_x_index).fu);
title('Comparison between Pu and fu');


