disp('RIGID case')

M_rob = M_rotor+M_link;

v_safe_rigid = get_v_from_HIC(HIC_max,M_rob, M_oper, K_cov);

[T_rigid,u_rigid] = RigidOptimizer(q0,q1,U_max,N,M_rob,v_safe_rigid);

dt_rigid = T_rigid/N;
fprintf(['\nResults (rigid):\n - T = %i\n - dt = %i\n - v safe = %f\n' ...
    '----------\n'],T_rigid,dt_rigid,v_safe_rigid)


%Get trajectory in time from u_rigid 
x_rigid = zeros(2,N);  %x(0)=(0,0)
dx_rigid = zeros(2,N); %dx(0)=(0,0)
x_rigid(1,1)=q0;
for i=1:N-1
    dx_rigid(1,i) = x_rigid(2,i); 
    dx_rigid(2,i) = u_rigid(i)/M_rob; % rotor acc

    x_rigid(:,i+1) = x_rigid(:,i)+dt_rigid.*dx_rigid(:,i); %Euler integration
end
  
HIC_rigid = zeros(1,N); 

for i=1:N
    HIC_rigid(:,i) = get_HIC_from_v(dx_rigid(1,i),M_rob,M_oper,K_cov);
end
timesteps_rigid = (T_rigid/N:T_rigid/N:T_rigid);

