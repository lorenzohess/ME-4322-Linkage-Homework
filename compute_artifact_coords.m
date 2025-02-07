clear; clc;

syms xa ya;
fx = -0.98; fy = 2.57;
gx = 0.05; gy = 0.2;
theta = atan((fy - gy)/(fx - gx));
e1 = cos(theta) == (xa - fx) / 1.843;
e2 = sqrt((xa - fx)^2 + (ya - fy)^2) == 1.843;
s = solve([e1, e2], [xa, ya]);
double(fx + s.xa) % -1.225
double(fy + s.ya) % choose 6.830 vs. 3.450
