init;
syms T real
N=10;
u_k = sym("u_k",[1,N]); assume(u_k,"real");
u_act = sym("u_act",[1,N]); assume(u_act,"real");


q0=0;
q1=1;
dt =T/N;

% x = zeros(4,N);  %x(0)=(0,0,0,0)
% dx = zeros(4,N); %dx(0)=(0,0,0,0)
x=[0,0,0,0]';


for i=2:N-1
    a_act_i = u_act(1,i);
    a_k_i = u_k(1,i);


    dx_1 = x(3)% rotor velocity
    dx_2= x(4)% link velocity
    dx_3 = (a_act_i-a_k_i*(x(1) - x(2)))/M_rotor % rotor acc
    dx_4 = a_k_i*(x(1) - x(2))/M_link % link acc
    dx = [dx_1,dx_2,dx_3,dx_4]';
    x=x+dt.*dx %Euler integration
    pause()
end

disp(x)



