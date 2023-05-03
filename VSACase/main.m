init;
addpath(".\utils")

fprintf('HIC_max = %d,   U_max = %d\n', ...
    HIC_max,U_max )
delta_q = q1-q0;
N = 100;

[T,uk_opt] = VSAOptimizer(delta_q,U_max,M_link,M_rotor,M_oper,K_cov,N,uk_bounds,HIC_max);

v_safe = zeros(1,N);
for i=1:N
    v_safe(:,i) = get_v_safe(uk_opt(i),M_link, M_rotor,M_oper,K_cov, HIC_max);
end
fprintf('%i is the Final Countdown\n',T)

[q,dq,ddq] = quintic_poli(q0,q1,N,T);

timesteps = (T/N:T/N:T);
figure
plot(timesteps,uk_opt)

user_in = input('Close all graphs?');
if user_in=='y'
    close all
end
