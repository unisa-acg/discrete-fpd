close all;
clear all;

b = 8e-5;
m = 0.5;
l = 0.2;

s = tf('s');

A = [0 1; 0 -b/m/l^2];
B = [0; 1/m/l^2];
C = [1 0];

P_s = C*(s*eye(2)-A)^(-1)*B;
C_s = 15/s * (5+s)^2 / (50+s);
%C_s = 1/s * (5+s)^2 / (50+s);

F_s = C_s * P_s;

figure;
bode(F_s);
grid on;

figure;
rlocus(F_s);

W_s = F_s/(1+F_s);

% Convert to digital

[num,den] = tfdata(C_s);
syms s z;
C_s = poly2sym(cell2mat(num),s)/poly2sym(cell2mat(den),s);

T_delta = 0.01;
C_z = simplify(subs(C_s, s, (z-1)/T_delta));

pretty(C_z);