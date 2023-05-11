function plotRigid(x,dx,ddx,u,timesteps,v_safe)

figure;
subplot(2,2,1)
plot(timesteps,x);
grid; title('rotor & link position (rigid)');
xlabel('[s]');ylabel('p[m]');
legend('rotor','link');

subplot(2,2,2)
plot(timesteps,dx,timesteps,v_safe,'g.');
grid; title('velocity (rigid)');xlabel('[s]');ylabel('v[m/s]')
legend('rotor','link','v_{safe}');


subplot(2,2,3)
plot(timesteps,ddx);
grid; title('link acceleration (rigid)');xlabel('[s]');ylabel('a[m/s^2]')
legend('rotor','link');

subplot(2,2,4)
plot(timesteps,u);grid; title('input (rigid)');xlabel('[s]');ylabel('u_{actuation}[?/s]')


end