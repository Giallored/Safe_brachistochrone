function [q] = cubic_poli(q0,dq0,q1,dq1,t,T)
delta_q = q1-q0;

%Coefficients enforcing the init/final conditions
c=dq0*T/delta_q;
b = (3-2*c-dq1/delta_q);
a=1-b-c;

q= q0+delta_q*(a*(t/T).^3 + b*(t/T).^2 + c*(t/T));
end
