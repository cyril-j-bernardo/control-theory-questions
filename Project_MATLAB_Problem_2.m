%% LINEAR CONTROL THEORY FINAL PROJECT QUESTION 2

% NOTE FOR THE GRADER: Each part that is written in code must be run within
% its own section. Some parts of the project have variables that rewrite
% themselves so if it is ran all at the same time you will receive
% conflicting information. Thank you!

clear; clc; 

r = 8.75 * 0.0254 %meters
J_b = 0.0049833 % kg*m^2
l_p = (13.125/2)* .0254 % meters
m_p = 0.126 % kg
g = 9.8 % meter/ s^2
R_a = 2.6 % Ohm
K_T = 0.00767 % N-m/Amp
K_b = 0.00767 % Volt-sec/rad
K_g = 14*4 %gear ratio

A = [0 0 1 0; 0 0 0 1; 0 (-m_p*r*g/(J_b)) 0 0 ; 0 (J_b + (m_p*r^2))/(l_p*J_b) 0 0]
B = [0;0;1/J_b;(-r/(l_p*J_b))]
C = [1 0 0 0;0 1 0 0]
D = [0;0]

sys = ss(A,B,C,D)
%% PROBLEM 2 PART B

poles = eig(A)
p1 = -1.2655e2
p2 = -0.0211e2
p3 = (-0.0248 + 0.0201*i)*10^2
p4 = (-0.0248 - 0.0201*i)*10^2

x0 = [pi/180;3*pi/180;0;0]

K_cont = place(A,B,[p1 p2 p3 p4])
sys_cl = ss(A-B*K_cont,B,C,D)
t = [0:0.01:10]
u = heaviside(t)
lsim(sys_cl,u,t,x0) 

%% PROBLEM 2 PART C

poles = [-25 -30 -35 -1000]

observgain = acker(A,B,poles)

