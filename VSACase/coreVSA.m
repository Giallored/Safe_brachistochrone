
disp('VSA case')

[T_VSA,uk_VSA,u_VSA] = VSAOptimizer(q0,q1,T_rigid,u_rigid, ...
    U_max,M_link,M_rotor,M_oper,K_cov,N,u_k_bounds,HIC_max,gamma);

dt_VSA = T_VSA/N;
fprintf('\nResults (VSA):\n - T = %i\n - dt = %i\n----------\n',T_VSA,dt_VSA)


%Get trajectory in time from u_act and uk

x_VSA = zeros(4,N);  
dx_VSA = zeros(4,N); 

for i=1:N-1
    dx_VSA(1,i) = x_VSA(3,i); % rotor velocity
    dx_VSA(2,i) = x_VSA(4,i); % link velocity
    dx_VSA(3,i) = (u_VSA(i)-uk_VSA(i)*(x_VSA(1,i) - x_VSA(2,i)))/M_rotor; % rotor acc
    dx_VSA(4,i) = uk_VSA(i)*(x_VSA(1,i) - x_VSA(2,i))/M_link; % link acc
    x_VSA(:,i+1) = x_VSA(:,i)+dt_VSA.*dx_VSA(:,i);
end

timesteps_VSA = (T_VSA/N:T_VSA/N:T_VSA);

%Get v_safe in time
v_safe_VSA = zeros(1,N);
HIC_VSA= zeros(1,N); 

for i=1:N
    M_rob = get_M_rob(uk_VSA(i),M_link,M_rotor,gamma);
    v_safe_VSA(:,i) = get_v_from_HIC(HIC_max,M_rob,M_oper,K_cov);

    HIC_VSA(:,i) = get_HIC_from_v(dx_VSA(2,i),M_rob,M_oper,K_cov);
    
end
    
