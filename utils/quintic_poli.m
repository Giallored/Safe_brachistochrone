function [q,dq,ddq] = quintic_poli(q0,q1,N,T)
%Quintic polinomial trajectory for a REST-TO-REST motion
% s = t/T;
syms s real
delta_q = q1-q0;
q = q0+delta_q*(6*(s)^5-15*(s)^4+10*(s)^3);
dq = delta_q/T*(30*s^2 - 60*s^3 + 30*s^4);
ddq = -delta_q/(T^2)*(60*s - 180*s^2 + 120*s^3);

p=zeros(1,N);
v=zeros(1,N);
a=zeros(1,N);

for k=1:N
    s_k = k/N;
    p(1,k)=subs(q,s,s_k);
    v(1,k)=subs(dq,s,s_k);
    a(1,k)=subs(ddq,s,s_k);

end

timesteps = (T/N:T/N:T);

figure
plot(timesteps,p)

figure
plot(timesteps,v)

figure
plot(timesteps,a)
end