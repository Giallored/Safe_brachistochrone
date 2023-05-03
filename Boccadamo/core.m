function core(Ktrasm,Ttobeat,granularity)


%close;
disp('Questo script calcola l''evoluzione di un sistema tipo VST');
disp('con un ingresso di tipo bang-zero-bang con istanti di commutazione');
disp(['parametrizzati al fine di portare il sistema nello zero,senza ' ...
    'violare i vincoli di HIC.']); 
disp('Premi un tasto per continuare...');

pause
clc;
close;

fprintf('Ttobeat = %f\n', Ttobeat);


%% UTILS 

Kriv = 5000;
Mope = 4;
Mrot = 1.2 ;
M1 = Mrot/2;
M2 = Mrot/2;
Mlink = 0.10005 ;
k = 1;
L0 = 0.7*pi ;
l0 =0;
N = ceil(Ttobeat/granularity);

init_pos= -2*pi;

initState = [init_pos-L0*sqrt(Ktrasm/k)/(2*sqrt(Ktrasm/k)+1),...
    init_pos,...
    init_pos+L0*sqrt(Ktrasm/k)/(2*sqrt(Ktrasm/k)+1),...
    0, 0, 0];

Umax = 30 ;
U1max = Umax /2;
U2max = Umax /2;
HICmax = 100 ;

save funUTILS.mat M1 M2 Mlink Ktrasm k L0 l0 initState HICmax Mope Kriv Ttobeat granularity N;



while isempty(Ttobeat)||Ttobeat <= 0

    Ttobeat = input('Inserisci il tempo da battere(in secondi): ');

end

fprintf('granularity = %f\n', granularity);

while isempty(granularity) || granularity == 0 || granularity >= Ttobeat
    
    granularity = input(['Inserisci la granularit`a del' ...
        'controllo piecewise constant: ']);

end



%% OPTIMIZATION PROBLEM

UU0 = zeros(1, 2*N);

LB = [ -U1max*ones(1, N) -U2max*ones(1, N) ];
UB = [U1max*ones(1,N) U2max*ones(1, N) ];

optimization = optimset('LargeScale', 'off', 'Diagnostic', 'on', 'MaxFunEvals', 10000, ...
'MaxIter', 10000, 'TolX', 1E-6);

[u, fval, exitflag] =fmincon(@myObjFun, UU0, [], [], [], [], LB, UB, ...
'myConstrFun', optimization);

if exitflag <= 0
    go = 0;
end

fprintf('End optimization at fval = %f\n',fval)



%% PLOT

close all;
load utils.mat;

timeV = [0:Ttobeat/(N+1):Ttobeat];

tempV = [0 constr 0]; 

if (length(tempV) > length(timeV))
    tempV = tempV(1:length(timeV));
elseif (length(tempV) < length(timeV))
    timeV = timeV(1:length(tempV));
end

tempV1 = [initState(5) speed 0]; 
if (length(tempV1) > length(timeV))
    tempV1 = tempV1(1:length(timeV));
elseif (length(tempV1) < length(timeV))
    timeV = timeV(1:length(tempV));
end

tempV2 = [0 u(1,1:N) 0]; 
if (length(tempV2) > length(timeV))
    tempV2 = tempV2(1:length(timeV));
elseif (length(tempV2) < length(timeV))
timeV = timeV(1:length(tempV2));
end

tempV3 = [0 u(1,N+1:2*N) 0];
if (length(tempV3)>length(timeV))
    tempV3 = tempV3(1:length(timeV));
elseif (length(tempV3) < length(timeV))
    timeV = timeV(1:length(tempV3));
end

subplot(2,1,1),
bar(timeV, tempV, 0.2), grid on, hold on;
plot([0 Ttobeat], [HICmax HICmax], 'r:');
title('HIC Values during task');
xlabel('Time');

subplot(2,1,2),plot(timeV, tempV1, 'b', 'LineWidth',1),grid on;
hold on, plot(timeV, tempV2, 'g--', 'LineWidth', 1);
plot(timeV, tempV3, 'm:', 'LineWidth', 1); xlabel('Time');
legend('Speed of link','Input torque1', 'Input torque2');

end


function F = myObjFun(UU)
    F = -UU(1)^2;
end
