init;
addpath(".\utils")

fprintf('v_safe = %d,   HIC = %d,   U_max = %d,   a_max = %d\n', ...
    v_safe,HIC_max,U_max, U_max/M_rob )
delta_q = q1-q0;
N = 1000;

optVar = rigidOptimizer(delta_q,v_safe,U_max,M_rob,N);
T = optVar(1);

fprintf('%i is the Final Countdown\n',T)

[q,dq,ddq] = quintic_poli(q0,q1,N,T);

user_in = input('Close all graphs?');
if user_in=='y'
    close all
end
