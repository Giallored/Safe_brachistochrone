function delta_Tk = optProblemRigid(M_rob,U_max,v_safe,aux_v_k,aux_a_k)

fun = @(dT) dT;

A=[];
b=[];
Aeq=[];
beq=[];
lb=0;
ub=inf;
% c_1 = @(dT) abs(aux_v_k)-v_safe*dT;
% c_2 = @(dT) abs(aux_a_k)-U_max/M_rob*dT^2;
% c = {c_1,c_2};
% ceq = {0,0};
nonlcon = @(dT) constraints(dT,aux_v_k,aux_a_k,v_safe,M_rob,U_max);
% nonlcon = [c,ceq];
dT0 = 10;

delta_Tk = fmincon(fun,dT0,A,b,Aeq,beq,lb,ub,nonlcon);
end


function [c,ceq] = constraints(dT,aux_v_k,aux_a_k,v_safe,M_rob,U_max)
    c(1)=double(abs(aux_v_k)-v_safe*dT);
    c(2)=double(abs(aux_a_k)-U_max/M_rob*dT^2);
    ceq=[];
end

