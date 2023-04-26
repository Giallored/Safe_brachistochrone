function objfun = objFunction(T)
Y = @(T,t) 1;
objfun = integral(@(t) Y(T,t),0,T,'ArrayValued',true);
end