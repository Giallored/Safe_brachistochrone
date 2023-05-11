function plotRigid(x,dx,ddx,u,timesteps,v_safe)

figure;
subplot(2,2,1)
plot(timesteps,x,'r-');
grid; title('link position (rigid)');
xlabel('[s]');ylabel('p[m]');
legend('link');

subplot(2,2,2)
plot(timesteps,dx,'r-',timesteps,v_safe,'g.');
grid; title('velocity (rigid)');xlabel('[s]');ylabel('v[m/s]')
legend('link','v_{safe}');


subplot(2,2,3)
plot(timesteps,ddx,'r-');
grid; title('link acceleration (rigid)');xlabel('[s]');ylabel('a[m/s^2]')
legend('link');

subplot(2,2,4)
plot(timesteps,u,'r-');grid; title('input (rigid)');xlabel('[s]');ylabel('u_{actuation}[?/s]')

% figure
% plot(timesteps,x,'g-');
% grid; title('link position (rigid)');
% xlabel('[s]');ylabel('p[m]');
% legend('link');
% 
% figure
% plot(timesteps,dx,'g-',timesteps,v_safe,'r*');
% grid; title('velocity (rigid)');xlabel('[s]');ylabel('v[m/s]')
% legend('link','v_safe');
% 
% figure
% plot(timesteps,ddx,'g-');
% grid; title('link acceleration (rigid)');xlabel('[s]');ylabel('a[m/s^2]')
% legend('link');
% 
% figure
% plot(timesteps,u,'b-');grid; title('input (rigid)');xlabel('[s]');ylabel('u_{actuation}[?/s]')


end