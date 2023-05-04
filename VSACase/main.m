init;
addpath(".\utils")
addpath("C:\Users\Adriano\Documents\AI & robotics\Elective\Elective_Deluca\Safe_brachistochrone\utils")

fprintf('HIC_max = %d,   U_max = %d\n', ...
    HIC_max,U_max )
delta_q = q1-q0;
N = 100;

[T,uk_opt,q_link,dq_link,ddq_link] = VSAOptimizer(delta_q,U_max,M_link,M_rotor,M_oper,K_cov,N,uk_bounds,HIC_max);


syms s real
[q,dq,ddq] = quintic_poli(s,q0,q1,N,T);

v_safe = zeros(1,N);
p=zeros(1,N);
v=zeros(1,N);
a=zeros(1,N);

for k=1:N
    v_safe(:,k) = get_v_safe(uk_opt(k),M_link, M_rotor,M_oper,K_cov, HIC_max);
    s_k = k/N;
    p(1,k)=subs(q,s,s_k);
    v(1,k)=subs(dq,s,s_k);
    a(1,k)=subs(ddq,s,s_k);
end
fprintf('%i is the Final Countdown\n',T)


timesteps = (T/N:T/N:T);

figure
plot(timesteps,p,'g-',timesteps,q_link,'b-');
grid; title('rotor position & link position');
xlabel('[s]');ylabel('p[m]');
legend('rotor','link');

figure
plot(timesteps,v,'g-',timesteps,dq_link,'b-',timesteps,v_safe,'r.' );
grid; title('rotor velocity, link velocity & safe velocity');xlabel('[s]');ylabel('v[m/s]')
legend('rotor','link','safe');

figure
plot(timesteps,a,'g-',timesteps,ddq_link,'b-');
grid; title('rotor acceleration & link acceleration');xlabel('[s]');ylabel('a[m/s^2]')
legend('rotor','link');

figure
plot(timesteps,uk_opt,'g-');grid; title('K trasmission');xlabel('[s]');ylabel('K_{trasmission}[?/s]')



user_in = input('Close all graphs?("n"for NO)');
if user_in =='n'
    disp('Closing maintaing the graphs.')
else
    close all force
    disp('Closing eliminating the graphs.')
end
