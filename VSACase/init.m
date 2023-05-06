clear all;

disp('Initializing...')

M_rotor = 5; %[kg*m^2]rotor inertia
M_link = 1; %[kg*m^2] link inertia
K_tras = 1000000; % K trasmission is initialized very stiff

M_oper = 10; % operator inertia
K_cov = 0.1; %stiffness of the coverage
U_max = 10; % torque max

HIC_max = 20;
uk_mean = 0.2;
duk = 0.8;

uk_bounds = [(1-duk)*uk_mean,(1+duk)*uk_mean];

q0 = 0;
q1= 2;
