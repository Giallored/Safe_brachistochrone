function [c, ceq] = myConstrFun(UU)

load funUTILS;

INTEGRATIONSTEP = 0.001;

inputNumber = 1;

T = 0.0;
rho = 0;

X = initState;

DX = [];
speed = [];
K = [];
constr = [];

alpha = 2^(5/2) * Kriv^(3/4) / (pi^(3/2) * Mope^(3/4));

while T < Ttobeat && ( (inputNumber+N) <= length(UU)) && (inputNumber <= N )

    Utau1 = UU(inputNumber);
    Utau2 = UU(inputNumber +N);
    
    DX(1) = X(4);
    
    DX(2) = X(5);
    
    DX(3) = X(6);
    
    DX(4) = (1/M1) * ( k*sign(X(2)-X(1)-l0)*(X(2)-X(1)-l0)^2-Ktrasm*sign(L0+X(1)-X(3))*...
    (L0+X(1)-X(3))^2+ Utau1 );

    DX(5) = (1/Mlink) * ( k*sign(X(2)-X(1)-l0)*(X(2)-X(1)-l0)^2+k*sign(X(3)-X(2)-l0)*...
        (X(3)-X(2)-l0)^2 );
    
    DX(6) = (1/M2) * ( -k*sign(X(3)-X(2)-l0)*(X(3)-X(2)-l0)^2+Ktrasm*sign(L0+X(1)-X(3))*...
        (L0+X(1)-X(3))^2 + Utau2 );

    % update
    DX = DX .* INTEGRATIONSTEP;
    X = X + DX;
    rho = rho + 1;
    T = rho*INTEGRATIONSTEP;

    if ~mod(T, granularity)

        inputNumber = inputNumber+1;
        speed = [speed X(5)];
        
        Mrobot = Mlink+(2*k*abs(X(1)-X(2))/(2*k*abs(X(1)-X(2))+1)*M1)+(2*k*abs(X(3)-X(2))/...
            (2*k*abs(X(3)-X(2))+1)*M2);
        
        temp01 = (Mrobot/(Mrobot+Mope))^(7/4);
        
        constr = [constr alpha*abs(X(5)^(5/2))*temp01];
    
    end

end

clear temp01;

c = constr-HICmax;
ceq = [X(2) X(5)];

save utils.mat speed c ceq constr;