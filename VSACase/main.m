init;
addpath(".\utils")
addpath("C:\Users\Adriano\Documents\AI & robotics\Elective\Elective_Deluca\Safe_brachistochrone\utils")

fprintf('HIC_max = %d,   U_max = %d\n', ...
    HIC_max,U_max )
N = 20;

[T,uk_opt,u_opt,q_rotor,dq_rotor,ddq_rotor] = VSAOptimizer(q0,q1,U_max, ...
    M_link,M_rotor,M_oper,K_cov,N,uk_bounds,HIC_max);

v_safe = zeros(1,N);
p=zeros(1,N);
v=zeros(1,N);
a=zeros(1,N);

for k=1:N
    v_safe(:,k) = get_v_safe(uk_opt(k),M_link, M_rotor,M_oper,K_cov, HIC_max);
    s_k = k/N;
    [p(1,k),v(1,k),a(1,k)] = quintic_poli(s_k,q0,q1,T);
end

fprintf('%i is the Final Countdown\n',T)


timesteps = (T/N:T/N:T);

figure
plot(timesteps,p,'g-',timesteps,q_rotor,'b*');
grid; title('rotor position & link position');
xlabel('[s]');ylabel('p[m]');
legend('link','rotor');

figure
plot(timesteps,v,'g-',timesteps,dq_rotor,'b-',timesteps,v_safe,'r.' );
grid; title('velocity');xlabel('[s]');ylabel('v[m/s]')
ylim([-10,100]);
legend('link','rotor','safe');

figure
plot(timesteps,a,'g-',timesteps,ddq_rotor,'b-');
grid; title('rotor acceleration & link acceleration');xlabel('[s]');ylabel('a[m/s^2]')
legend('link','rotor');

figure
plot(timesteps,uk_opt,'g-');grid; title('K trasmission');xlabel('[s]');ylabel('K_{trasmission}[?/s]')

figure
plot(timesteps,u_opt,'b-');grid; title('input');xlabel('[s]');ylabel('u_{rotor}[?/s]')


user_in = input('Close all graphs?("n"for NO)');
if user_in =='n'
    disp('NOT closing the graphs.')
else
    close all force
    disp('Closing the graphs.')
end
