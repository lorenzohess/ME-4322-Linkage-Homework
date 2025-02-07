clear; clc;

% Artifact
syms xa ya;
fx = -0.98; fy = 2.57;
gx = 0.05; gy = 0.2;
theta = atan((fy - gy)/(fx - gx));
e1 = cos(theta) == (xa - fx) / 1.843;
e2 = sqrt((xa - fx)^2 + (ya - fy)^2) == 1.843;
s = solve([e1, e2], [xa, ya]);
double(fx + s.xa) % -1.225
double(fy + s.ya) % choose 3.450 vs. 6.830

% Com CDE
xd = 0.285; yd = 0.055;
ye = 2.54;
t = asin((ye - yd) / 2.4866);

xs = 1.2463 * cos(t) + xd
ys = 1.2463 * sin(t) + yd
