init;


T_ottimoVST = []; 

for i = 1:length(Kvect)

    Ktrasm = Kvect(i);
    tau = tauvect(i);
    
    T_tobeat = 3.1;
    dt = 0.05;
    go = 1;
    
    while go == 1
        
        fprintf('Ktrasm = %f\n', Ktrasm);
    
        core(Ktrasm,T_tobeat, dt);
    
        pause(1);
    
        T_tobeat = T_tobeat -  dt;
    
    end
    
    T_ottimoVST = [T_ottimoVST T_tobeat+ dt];
    
    pause(1);

end 
plot(Kvect, T_ottimoVST, 'c:', 'LineWidth', 2);
xlabel('Valore di K* [N/mm^2]');
ylabel('Tempo ottimo [s]');