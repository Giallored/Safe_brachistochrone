function T = optProblemRigid(M_rob,U_max,v_safe,q0,dq0,q1,dq1)

% Objective function
Y =@(t,T) 1;
objfun =@(T) integral(@(t) Y(t,T),0,T,'ArrayValued',true);

T0=100;
 
traj =@(t,T) cubic_poli(t,T,q0,dq0,q1,dq1);

%velocity constraint
dq =@(t,T) diff(traj(t,T),t);
dq_fun =@(t,T) dq(t,T)-v_safe;

% input constraint
ddq =@(t,T) diff(dq(t,T),t);
u_fun = @(t,T) M_rob*ddq(t,T)-U_max;

% c = [dq_fun;u_fun];
% c_eq = None;


[T,fval] = fmincon(objfun,T0,[],[],[],[],0,inf,dq_fun);


end

function [c,c_eq] = getConstraints(t,T,v_safe,traj)

%velocity constraint
dq = diff(traj,t);
dq_fun =@(t,T) dq-v_safe;

% input constraint
ddq = diff(dq,t);
u_fun = @(t,T) M_rob*ddq(t,T)-U_max;

c = [dq_fun;u_fun];
c_eq = None;


end