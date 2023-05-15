function [T_opt,uk_opt,u_opt] = VSAOptimizer(q0,q1,T_rigid,u_rigid, ...
    U_max,M_link,M_rotor,M_oper,K_cov,N,uk_bounds,HIC_max,gamma)

% variables:
% - T
% - uk_i  forall i=1,...,N: K_trasmission is an input
% - u_act_i  forall i=1,...,N: motor input
N_vars = 1+2*N; %number of variables 

A=[];
b=[];
Aeq=[];
beq=[];


% Boundaries
lb = -ones(1,N_vars)*inf;
ub = ones(1,N_vars)*inf;
lb(1)=0; %time bounds
lb(2:N+1)=uk_bounds(1);% stiff bounds
ub(2:N+1)=uk_bounds(2);
lb(N+2:2*N+1)=-U_max;% 
ub(N+2:2*N+1)=U_max;

%initial conditions
var0 = zeros(1,N_vars);
var0(1) = T_rigid; %time
var0(2:N+1)=0;
var0(N+2:2*N+1)=u_rigid;


objfun = @(var) var(1);
nonlcon = @(var) constraints(var,N,q0,q1,M_rotor,M_link,M_oper,K_cov,HIC_max,gamma);

optimization_parms = optimset(  'MaxFunEvals', 1000000, ...
'MaxIter', 10000, 'TolX', 1E-6);%,'Diagnostic', 'on');

[varOpt,fvalue] = fmincon(objfun,var0,A,b,Aeq,beq,lb,ub,nonlcon,optimization_parms);

T_opt =   varOpt(1);
uk_opt =   varOpt(2:N+1);
u_opt = varOpt(N+2:2*N+1);


end

function [c,ceq] = constraints(var,N,q0,q1,M_rotor,M_link,M_oper,K_cov,HIC_max,gamma)
    
    % get the different opt. variables
    T = var(1);
    u_k = var(2:N+1);
    u_act = var(N+2:2*N+1);

    % get the parameters
    dt =T/N;

    if dt<0.001
        dt_int=dt;
        MCD=1;
    else
        MCD = floor(dt/0.001)+1;
        dt_int=dt/MCD;
    end

    % Define the trajectory configuration vars as:
    % - x_i = (q_i,dq_i), 
    % - dx_i = (dq_i,ddq_i)
    % where q = (q_rotor, q_link)
    x = zeros(4,N);  %x(0)=(0,0,0,0)
    dx = zeros(4,N); %dx(0)=(0,0,0,0)
    x(1,1)=q0;
    x(2,1)=q0;

    x_j=[0 0 0 0]';
    dx_j = [0 0 0 0]';
    if false
        for i=1:N-1
            for j=1:MCD
                dx_j(1) = x_j(3); % rotor velocity
                dx_j(2) = x_j(4); % link velocity
                dx_j(3) = u_act(i) - u_k(i)*(x_j(1) - x_j(2))/M_rotor; % rotor acc
                dx_j(4) = u_k(i)*(x_j(1) - x_j(2))/M_link; % link acc
                x_j = x_j+dt_int.*dx_j; %Euler integration
            end
    
            x(:,i)=x_j;
            dx(:,i)=dx_j;
            x(:,i+1) = x_j;
        end
    else
    
        for i=1:N-1
            dx(1,i) = x(3,i); % rotor velocity
            dx(2,i) = x(4,i); % link velocity
            dx(3,i) = (u_act(i)-u_k(i)*(x(1,i) - x(2,i)))/M_rotor; % rotor acc
            dx(4,i) = u_k(i)*(x(1,i) - x(2,i))/M_link; % link acc
    
            x(:,i+1) = x(:,i)+dt.*dx(:,i); %Euler integration
        end
    end 
    
    % init/terminal conditions 
    ceq_init(1,1) =x(1,N)-q1;
    ceq_init(1,2) = x(2,N)-q1;
    ceq_init(1,3) = x(3,N);
    ceq_init(1,4) =x(4,N);
     
     % Def. Constraint
    for i=1:N
        %contraints on link velocity to be safe
        M_rob_i =  get_M_rob(u_k(i),M_link,M_rotor,gamma);
         
        v_safe_i = get_v_from_HIC(HIC_max, M_rob_i,M_oper,K_cov);
        
        c_dq(1,i) = abs(dx(2,i)) -v_safe_i;

        % fixing stiffness
        %ceq_k(1,i) = u_k(i) - 1e5;
    end 
    c =c_dq;
    ceq =ceq_init;% [ceq_k,ceq_init];
end


