clear all;

disp('Initializing...')

M_rotor = 5; %[kg*m^2]rotor inertia
M_link = 1; %[kg*m^2] link inertia
M_rob = M_rotor+M_link;
M_oper = 10; % operator inertia
K_cov = 0.1; %stiffness of the coverage
U_max = 1000; % torque max
a_max = U_max/M_rob;
HIC_max = 20;
v_safe = get_v_from_HIC(HIC_max,M_rob,M_oper,K_cov);

q0 = 0;
q1= 100;
dq0= 0;
dq1= 0;