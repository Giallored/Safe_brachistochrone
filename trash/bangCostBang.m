function [q,dq,ddq] = bangCostBang(v_max,a_max,L,N,T)

timesteps = (0:T/N:T)
disp(timesetps)

syms t real
q_sym(1) = a_max*t^2/2;
q_sym(2) = v_max*t + v_max^2/2*a_max;
q_sym(3) = -a_max*(t-T)^2/2+v_max*T-v_max^2/a_max;
dq_sym=diff(q_sym,t);
Ts = v_max/a_max;

q =zeros(N,1);
dq =zeros(N,1);
ddq =zeros(N,1);

if true%L> v_max^2/a_max %Bang-Bang case
    for time=1:size(timesteps,2)
        if time<Ts
            fprintf('---BANG 1 -> %i\n',time)

            q(time,1) =  subs(q_sym(1),t,time);
            dq(time,1) = subs(dq_sym(1),t,time);
            ddq(time,1) = a_max;
        end
        if time>= Ts && time<T-Ts
            fprintf('---COST -> %i\n',time)
            q(time,1) = subs(q_sym(2),t,time);
            dq(time,1) = v_max;
            ddq(time,1) = 0;
        end
        if time>= T-Ts
            fprintf('---BANG 2 -> %i\n',time)

            q(time,1) = subs(q_sym(3),t,time);
            dq(time,1) = subs(dq_sym(3),t,time);
            ddq(time,1) = -a_max;
        end
    end
end
figure
plot(timesteps,q)

figure
plot(timesteps,dq)

figure
plot(timesteps,dq)


end