init;
addpath(".\utils")
addpath("C:\Users\Adriano\Documents\AI & robotics\Elective\Elective_Deluca\Safe_brachistochrone\utils")

fprintf('HIC_max = %d,   U_max = %d\n', ...
    HIC_max,U_max )

N = 100;

q0 = 0;
q1= 1;

[T,uk,u_act,q_rotor,dq_rotor,q_link,dq_link] = VSAOptimizer(q0,q1,U_max, ...
    M_link,M_rotor,M_oper,K_cov,N,uk_bounds,HIC_max,gamma);


v_safe = zeros(1,N);
ddq_rotor = zeros(1,N);
ddq_link = zeros(1,N);
for i=1:N
    v_safe(:,i) = get_v_safe(uk(i),M_link, M_rotor,M_oper,K_cov, HIC_max);
    ddq_rotor(1,i)= (u_act(i)-uk(i)*(q_rotor(i) - q_link(i)))/M_rotor;
    ddq_link(1,i)= uk(i)*(q_rotor(i) - q_link(i))/M_link;
end

fprintf('%i is the Final Countdown\n',T)


timesteps = (T/N:T/N:T);

figure
plot(timesteps,q_link,'g-',timesteps,q_rotor,'b-');
grid; title('rotor position & link position');
xlabel('[s]');ylabel('p[m]');
legend('link','rotor');

figure
plot(timesteps,dq_link,'g-',timesteps,dq_rotor,'b-',timesteps,v_safe,'r.' );
grid; title('velocity');xlabel('[s]');ylabel('v[m/s]')
ylim([-10,100]);
legend('link','rotor','safe');

figure
plot(timesteps,ddq_link,'g-',timesteps,ddq_rotor,'b-');
grid; title('rotor acceleration & link acceleration');xlabel('[s]');ylabel('a[m/s^2]')
legend('link','rotor');

figure
plot(timesteps,uk,'g-');grid; title('K trasmission');xlabel('[s]');ylabel('K_{trasmission}[?/s]')

figure
plot(timesteps,u_act,'b-');grid; title('input');xlabel('[s]');ylabel('u_{actuation}[?/s]')


user_in = input('Close all graphs?("n"for NO)');
if user_in =='n'
    disp('NOT closing the graphs.')
else
    close all force
    disp('Closing the graphs.')
end
