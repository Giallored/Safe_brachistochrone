clear all;

disp('Initializing...')

M_rotor = 1.2; %[kg*m^2]rotor inertia
M_link = 0.1; %[kg*m^2] link inertia
M_oper = 4; % operator inertia
K_cov = 5000; %stiffness of the coverage
U_max = 30; % torque max

HIC_max = 0;
while HIC_max <1
    HIC_max = input('HIC max (only pos numbers)? -> ');
end

u_k_mean =1e3;
du_k = 0.8;
gamma = 3e3;

u_k_bounds = [(1-du_k)*u_k_mean,(1+du_k)*u_k_mean];



%GOOD PARAMS
% M_rotor = 1.2; %[kg*m^2]rotor inertia
% M_link = 0.1; %[kg*m^2] link inertia
% K_tras = 1; % K trasmission is initialized very stiff
% M_oper = 4; % operator inertia
% K_cov = 5000; %stiffness of the coverage
% U_max = 30; % torque max
% 
% u_k_mean =1e3;
% du_k = 0.8;
% gamma = 3e3;

% u_k_mean =1e2;
% du_k = 0.8;
% gamma = 3e2;

