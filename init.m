clear all;

disp('Initializing...')

M_rotor = 1.2; %[kg*m^2]rotor inertia
M_link = 0.1; %[kg*m^2] link inertia
M_oper = 10; % operator inertia
K_cov = 0.1; %stiffness of the coverage
U_max = 15; % torque max

q0 = 0;
q1= 1;
dq0= 0;
dq1= 0;