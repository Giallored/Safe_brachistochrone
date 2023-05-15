
function [T_opt,u_opt] = RigidOptimizer(q0,q1,U_max,N,M_rob,v_safe)

% variables:
% - T
% - uk_i  forall i=1,...,N: K_trasmission is an input
% - u_act_i  forall i=1,...,N: motor input
N_vars = 1+N; %number of variables 

A=[];
b=[];
Aeq=[];
beq=[];


% Boundaries
lb = -ones(1,N_vars)*inf;
ub = ones(1,N_vars)*inf;
lb(1)=0; %time bounds

lb(2:N+1)=-U_max;
ub(2:N+1)=U_max;

%initial conditions
var0 = zeros(1,N_vars);
var0(1) = 10; %time
var0(2:N+1)=0.4;


objfun = @(var) var(1);
nonlcon = @(var) constraints(var,N,q0,q1,M_rob,v_safe);

optimization_parms = optimset(  'MaxFunEvals', 1000000, ...
'MaxIter', 10000, 'TolX', 1E-6);%,'Diagnostic', 'on');

[varOpt,fvalue] = fmincon(objfun,var0,A,b,Aeq,beq,lb,ub,nonlcon,optimization_parms);

T_opt =   varOpt(1);
u_opt = varOpt(2:N+1);
end

function [c,ceq] = constraints(var,N,q0,q1,M_rob,v_safe)
    
    % get the different opt. variables
    T = var(1);
    u_act = var(2:N+1);

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
    x = zeros(2,N);  %x(0)=(0,0)
    dx = zeros(2,N); %dx(0)=(0,0)
    x(1,1)=q0;
    x_j=[0,0]';
    dx_j=[0,0]';
    if false
        for i=1:N-1
            for j=1:MCD
                dx_j(1) = x_j(2); 
                dx_j(2) = u_act(i)/M_rob; % rotor acc
                
                x_j = x_j+dt_int.*dx_j; %Euler integration
            end
    
            x(:,i)=x_j;
            dx(:,i)=dx_j;
            x(:,i+1) = x_j;
        end
    else
        for i=1:N-1
            dx(1,i) = x(2,i); 
            dx(2,i) = u_act(i)/M_rob; % rotor acc
    
            x(:,i+1) = x(:,i)+dt.*dx(:,i); %Euler integration
        end
     end
    
    % init/terminal conditions 
    ceq_init(1,1) =x(1,N)-q1;
    ceq_init(1,2) = x(2,N);

    % Def. Constraint
    for i=1:N
        v = abs(x(2,i));
        c_dq(1,i) = v-v_safe;
    end 
    c =c_dq;
    ceq = ceq_init;
end 