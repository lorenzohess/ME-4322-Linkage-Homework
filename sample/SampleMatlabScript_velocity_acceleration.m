clc
clear

%joint coordinates

A=[7 4 0];
B=[5 16 0];
C=[25 25 0];
D=[23 10 0];
E=[18 35 0];
F=[43 32 0];
G=[45 17 0];

%position vectors
posAB=B-A;
posBC=C-B;
posBE=E-B;
posCD=D-C;
posEF=F-E;
posGF=F-G;

syms wBEC wCD 
wAB=[0 0 1];
omegaBEC=[0 0 wBEC];
omegaCD=[0 0 wCD];

eqn1=cross(wAB,posAB)+cross(omegaBEC,posBC)+cross(omegaCD,posCD)==0;

solution=solve(eqn1,[wBEC,wCD]);

w_BEC=double(solution.wBEC)
w_CD=double(solution.wCD)

omega_BEC=[0 0 w_BEC];
omega_CD=[0 0 w_CD];


%Angular Acceleration calculation

syms aBEC aCD
aAB=[0 0 0];
alphaBEC=[0 0 aBEC];
alphaCD=[0 0 aCD];

eqn2=cross(aAB,posAB)+cross(wAB, cross(wAB,posAB))+cross(alphaBEC,posBC)+cross(omega_BEC, cross(omega_BEC,posBC))+cross(alphaCD,posCD)+cross(omega_CD, cross(omega_CD,posCD))==0;

solution1=solve(eqn2,[aBEC,aCD]);

a_BEC=double(solution1.aBEC)
a_CD=double(solution1.aCD)

alpha_BEC=[0 0 a_BEC];
alpha_CD=[0 0 a_CD];


