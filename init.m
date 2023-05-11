clear all;

disp('Initializing...')
scale=1000;

M_rotor = 1.2*scale; %[kg*m^2]rotor inertia
M_link = 0.1*scale; %[kg*m^2] link inertia
K_tras = 1; % K trasmission is initialized very stiff
M_oper = 0.6;%1.2; % operator inertia
K_cov = 0.1; %stiffness of the coverage
U_max = 100*scale/5; % torque max

HIC_max = 0;
while HIC_max <10
    HIC_max = input('HIC max (only pos numbers)? -> ');
end

u_k_mean = 0.2;
du_k = 0.8;
gamma = 300;

u_k_bounds = [0,1e10];%[(1-du_k)*u_k_mean,(1+du_k)*u_k_mean];

