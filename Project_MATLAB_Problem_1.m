%% LINEAR CONTROL THEORY FINAL PROJECT QUESTION 1

% NOTE FOR THE GRADER: Each part that is written in code must be run within
% its own section. Some parts of the project have variables that rewrite
% themselves so if it is ran all at the same time you will receive
% conflicting information. Thank you!

clear; clc;

%%%%%%%%% CONSTANTS
J = 1; % kg/m^2
L = 1; % henry
R = 2; % ohm
damp = 1; % damping, N-m-s
k_T = 2; % N-m/A


%%%%%%%%%% STATE SPACE
A = [0 1 0; 0 (-damp/J) (k_T/J); 0 0 (-R/L)]
B = [0; 0;(1/L)]
C = [1 0 0]
D = 0

sys = ss(A,B,C,D)

%% PROBLEM 1 PART C

[b,a] = ss2tf(A,B,C,D)

%%% b represents the coefficients of the numerator
%%% a represents the coefficients of the denominator

%% PROBLEM 1 PART E
A_1 = [0 1 0; 0 (-damp/J) (k_T/J);0 0 (-R/L)]
A_2 = [0 1 0; 0 0 1; 0 (-damp*R/(J*L)) (-(damp*L + J*R)/(J*L))]

[Q_1,D_1] = eig(A_1)
[Q_2,D_2] = eig(A_2)

%% PROBLEM 1 PART H

clear; clc; 

J = 1; % kg/m^2
L = 1; % henry
R = 2; % ohm
damp = 1; % damping, N-m-s
k_T = 2; % N-m/A

A = [0 1 0; 0 (-damp/J) (k_T/J); 0 0 (-R/L)]
B = [0; 0;(1/L)]
C = [1 0 0]
D = 0

sys = ss(A,B,C,D)

Co = ctrb(sys)
Ob = obsv(sys)

%% PROBLEM 1 PART K

A = [0 1 0; 0 -1 2; 0 0 -2]
B = [0;0;1]

S = [B A*B (A^2)*B]
S_inv = inv(S)
M_1 = S_inv(rank(S_inv),:)
M = [M_1; (M_1)*A; (M_1)*A^2]
M_inv = inv(M)

A_con = M*A*M_inv
B_con = M*B
C_con = C*M_inv
D_con = 0

%% PROBLEM 1 PART L

% part A

A_A = [0 1 0; 0 -1 2; 0 0 -2]
B_A = [0;0;1]
C_A = [1 0 0]
D_A = 0

sys_A = ss(A_A,B_A,C_A,D_A)


y_A = step(sys_A,8)
x_A = [0:8/(size(y_A,1)-1):8]

figure
plot(x_A,y_A)

title('Part L - Original State Space Model')
xlabel('time')
ylabel('System response')

% part E

csys = canon(sys,'modal',Inf)
[A_diag,B_diag,C_diag,D_diag] = ssdata(csys)

y_diag = step(csys,8)
x_diag = [0: 8/(size(y_diag,1)-1):8]

figure
plot(x_diag,y_diag)
title('Part L - Diagonal Matrix')
xlabel('time')
ylabel('System Response')

% part K - Controller Canonical Form

sys_can = ss(A_con,B_con,C_con,D_con)

y_can = step(sys_can,8)
x_can = [0:8/(size(y_can,1)-1):8]

figure
plot(x_can,y_can)
title('Part L - Controller Canonical Form')
xlabel('time')
ylabel('System Response')

figure
plot(x_A,y_A,x_diag,y_diag,x_can,y_can)
title('Combined graphs')
xlabel('time')
ylabel('System Response')

%% PART M

% original state space form
poles = eig(A_A)
p1 = -4
p2 = -12
p3 = -13

K_A = place(A,B,[p1 p2 p3])
sys_A_cl = ss(A_A-B_A*K_A,B_A,C,0)

y_A_cl = step(sys_A_cl,8)
x_A_cl = [0:8/(size(y_A_cl,1)-1):8]

figure
plot(x_A_cl,y_A_cl)
title('Part M - Original State Space Form')
xlabel('time')
ylabel('System Response')

% Diagonal Matrix form - Closed
K_diag = place(A_diag,B_diag,[p1 p2 p3])
sys_diag_cl = ss(A_diag-B_diag*K_diag,B_diag,C,0)
y_diag_cl = step(sys_diag_cl,8)
x_diag_cl = [0:8/(size(y_diag_cl,1)-1):8]

figure
plot(x_diag_cl,y_diag_cl)
title('Part M - Diagonal State Space Form')
xlabel('time')

% Controller Canonical Matrix form - Closed
K_con = place(A_con,B_con,[p1 p2 p3])
sys_con_cl = ss(A_con-B_con*K_con,B_con,C,0)
y_con_cl = step(sys_con_cl,8)
x_con_cl = [0:8/(size(y_con_cl,1)-1):8]

figure
plot(x_con_cl,y_con_cl)
title('Part M - Controller Canonical State Space Form')
xlabel('time')
ylabel('System Response')

