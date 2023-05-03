function [T_opt,uk_opt] = VSAOptimizer(delta_q,U_max,M_link,M_rotor,M_oper,K_cov,N,uk_bounds,HIC_max)

% variables:
% - T
% - uk_i  forall i=1,...,N: K_trasmission is an input
% - q_i  ...... : position on the link
% - dq_i  ...... : velocity on the link
% - ddq_i  ...... : acceleration on the link
% resulting in var =
% [T,uk_1,...,uk_N,q_1...,q_N,dq_1...,dq_N,ddq_1...,ddq_N]

N_vars = 1+4*N; %number of variables 


A=[];
b=[];
Aeq=[];
beq=[];
lb = -ones(1,N_vars)*inf;
ub = ones(1,N_vars)*inf;


% Boundaries
lb(1)=0; %time bounds
lb(:,2:N+1)=uk_bounds(1);
ub(:,2:N+1)=uk_bounds(2);

%initial conditions
var0 = zeros(1,N_vars);
var0(1) = 10; %time

objfun = @(var) var(1);
nonlcon = @(var) constraints(var,N,delta_q,U_max,M_rotor,M_link,M_oper,K_cov,HIC_max);

varOpt = fmincon(objfun,var0,A,b,Aeq,beq,lb,ub,nonlcon);
T_opt =   varOpt(1);
uk_opt =   varOpt(2:N+1);
q_link =   varOpt(N+2:2*N+1);
dq_link =   varOpt(2*N+2:3*N+1);
ddq_link =   varOpt(3*N+2:4*N+1);


end

function [c,ceq] =  constraints(var,N,delta_q,U_max,M_rotor,M_link,M_oper,K_cov,HIC_max)
    T = var(1);
    uk = var(2:N+1);
    q_link = var(N+2:2*N+1);
    dq_link = var(2*N+2:3*N+1);
    ddq_link = var(3*N+2:4*N+1);
    

    % velcity constraints
    for i=1:N
        v_safe_i = get_v_safe(uk(i),M_link, M_rotor,M_oper,K_cov, HIC_max);
        c(i) = abs(dq_link(i))-v_safe_i;     %dq
    end


    for i=1:N
        s = i/N;
        q_i = delta_q*(6*(s)^5-15*(s)^4+10*(s)^3);
        ddq_i = -delta_q/T^2*(60*s - 180*s^2 + 120*s^3);
        u_act_i = M_rotor *ddq_i+uk(i)*(q_i-q_link(i));
        
        c(N+i) = abs(u_act_i)-U_max;   %ddq
    end


    for i=1:N
        s = i/N;
        q_i = delta_q*(6*(s)^5-15*(s)^4+10*(s)^3);
        
        ceq(i) = M_link *ddq_link(i)-uk(i)*(q_i-q_link(i));
    end
end


