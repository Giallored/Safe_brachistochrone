function T_tot = rigidOptimizer(delta_q,v_safe,U_max,M_rob,N)

A=[];
b=[];
Aeq=[];
beq=[];
lb=[0,0];
ub=[inf,1];

nonlcon = @(var) constraints(var,N,delta_q,v_safe,U_max,M_rob);

T0 = 10;
objfun = @(T) T;
T_tot = fmincon(objfun,T0,A,b,Aeq,beq,lb,ub,nonlcon);

end

function [c,ceq] = constraints(T,N,delta_q,v_safe,U_max,M_rob)
    for i=1:N
        s = i/N;
        c(i) = delta_q*(30*s^2 - 60*s^3 + 30*s^4)-v_safe*T;     %dq
    end
    for i=1:N
        s = i/N;
        c(N+i) = -delta_q*(60*s - 180*s^2 + 120*s^3)*M_rob-U_max*(T^2);   %ddq
    end
    ceq = [];
end

function fun = objectFunction(T)
    fun = T;
end
