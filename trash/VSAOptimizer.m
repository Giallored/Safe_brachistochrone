function [T_opt,uk_opt,u_opt,q_rotor,dq_rotor,q_link,dq_link] = VSAOptimizer(q0,q1, ...
    U_max,M_link,M_rotor,M_oper,K_cov,N,uk_bounds,HIC_max,gamma)

% variables:
% - T
% - uk_i  forall i=1,...,N: K_trasmission is an input
% - u_act_i  forall i=1,...,N: motor input
% - q_i  ...... : position on the rotor
% - dq_i  ...... : velocity on the rotor
% - ddq_i  ...... : acceleration on the rotor
% resulting in var =
% [T,uk_1,...,uk_N,q_1...,q_N,dq_1...,dq_N,ddq_1...,ddq_N]

N_vars = 1+6*N; %number of variables 

A=[];
b=[];
Aeq=[];
beq=[];
lb = -ones(1,N_vars)*inf;
ub = ones(1,N_vars)*inf;

% Boundaries
lb(1)=0; %time bounds
%ub(1)=5; %time bounds
lb(2:N+1)=uk_bounds(1);
ub(2:N+1)=uk_bounds(2);
lb(N+2:2*N+1)=-U_max;
ub(N+2:2*N+1)=U_max;

%initial conditions
var0 = zeros(1,N_vars);
var0(1) = 10; %time

uk_init = uk_bounds(2);
objfun = @(var) var(1);
nonlcon = @(var) constraints(var,N,q0,q1,M_rotor,M_link,M_oper,K_cov,HIC_max,gamma);

optimization = optimset('LargeScale', 'off', 'Diagnostic', 'on', 'MaxFunEvals', 100000, ...
'MaxIter', 10000, 'TolX', 1E-6);

[varOpt,fvalue] = fmincon(objfun,var0,A,b,Aeq,beq,lb,ub,nonlcon,optimization);

T_opt =   varOpt(1);
uk_opt =   varOpt(2:N+1);
u_opt = varOpt(N+2:2*N+1);
q_rotor = varOpt(2*N+2:3*N+1);
dq_rotor = varOpt(3*N+2:4*N+1);
q_link = varOpt(4*N+2:5*N+1);
dq_link = varOpt(5*N+2:6*N+1);

fprintf('Resulting dT = %f\n', T_opt/N)

end

function [c,ceq] = constraints(var,N,q0,q1,M_rotor,M_link,M_oper,K_cov,HIC_max,gamma)

    % get the different opt. variables
    T = var(1);
    uk = var(2:N+1);
    u_act = var(N+2:2*N+1);
    q_rotor = var(2*N+2:3*N+1);
    dq_rotor = var(3*N+2:4*N+1);
    q_link = var(4*N+2:5*N+1);
    dq_link = var(5*N+2:6*N+1);

    % get the parameters
    dt =T/N;

    % init/terminal conditions
    ceq_init(1,1) = q_rotor(1)-q0;
    ceq_init(1,2) = q_rotor(N)-q1;
    ceq_init(1,3) = dq_rotor(1);
    ceq_init(1,4) = dq_rotor(N); 
    ceq_init(1,5) = q_link(1)-q0;
    ceq_init(1,6) = q_link(N)-q1;
    ceq_init(1,7) = dq_link(1);
    ceq_init(1,8) = dq_link(N); 

  
    %find accelerations from dynamics
    ddq_rotor = zeros(1,N);
    ddq_link = zeros(1,N);
    for i=1:N
        ddq_rotor(1,i)= (u_act(i)-uk(i)*(q_rotor(i) - q_link(i)))/M_rotor;
        ddq_link(1,i)= uk(i)*(q_rotor(i) - q_link(i))/M_link;
    end
   
    
    %Interconnect the kinematics of the rotor trajectory using euler integ.

    for i=1:N-1
        % Rotor stuff
        dq_r_mean = (dq_rotor(i) + dq_rotor(i+1))/2;
        ddq_r_mean = (ddq_rotor(i) + ddq_rotor(i+1))/2;
        ceq_q_r(1,i) = q_rotor(i+1) - (q_rotor(i) + dt*dq_r_mean);
        ceq_dq_r(1,i) = dq_rotor(i+1) - (dq_rotor(i) + dt*ddq_r_mean);
        
        % Link stuff
        dq_l_mean = (dq_link(i) + dq_link(i+1))/2;
        ddq_l_mean = (ddq_link(i) + ddq_link(i+1))/2;
        ceq_q_l(1,i) = q_link(i+1) - (q_link(i) + dt*dq_l_mean);
        ceq_dq_l(1,i) = dq_link(i+1) - (dq_link(i) + dt*ddq_l_mean);
        
        %Every instant, q_link>=q_rotor
        c_q(1,i) = q_rotor(1,i)-q_link(1,i);

    end

    %constraints on link velocity
    for i=1:N
        M_rob_i = M_link + uk(i)/(uk(i)+gamma)*M_rotor;
        v_safe_i = get_v_from_HIC(HIC_max, M_rob_i,M_oper,K_cov);
        c_dq(1,i) = abs(dq_link(1,i))-v_safe_i;     
    end

    c =[c_q,c_dq] ;
    ceq = [ceq_init, ceq_q_r,ceq_dq_r,ceq_q_l,ceq_dq_l];



%     %Get the link trajectory as POLYNOMIAL 
%     q_link = zeros(1,N);
%     dq_link = zeros(1,N);
%     ddq_link = zeros(1,N);
%     for i=1:N
%         s=i/N;
%         [q_link(1,i),dq_link(1,i),ddq_link(1,i)] = quintic_poli(s,q0,q1,T);
%     end


%     gamma =1;
%     for i=1:N
%         M_rob = M_link + uk(i)/(uk(i)+gamma)*M_rot;
%         v_safe_i = get_v_from_HIC(HIC_max, M_rob,M_oper,K_cov);
%         c(1,i) = abs(dq_link(1,i))-v_safe_i;     %dq
%     end
%     for i=1:N
%         u_act_i = M_rotor *ddq_rotor(i)+uk(i)*(q_rotor(i)-q_link(1,i));
%         c(1,N+i) = abs(u_act_i)-U_max;   %ddq
%         ceq(1,2*N+i) = M_link *ddq_link(1,i)-uk(i)*(q_rotor(i)-q_link(1,i));
%     end
end


