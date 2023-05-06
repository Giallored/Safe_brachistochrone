function [q,dq,ddq] = quintic_poli(s,q0,q1,T)
%Quintic polinomial trajectory for a REST-TO-REST motion
% s = t/T;
% syms s real
delta_q = q1-q0;
q = q0+delta_q*(6*(s)^5-15*(s)^4+10*(s)^3);
dq = delta_q/T*(30*s^2 - 60*s^3 + 30*s^4);
ddq = -delta_q/(T^2)*(60*s - 180*s^2 + 120*s^3);

end