function HIC = get_HIC_from_v(v,M_rob, M_oper, K_cov)
    
    beta = 2*(2/pi)^(3/2)*(K_cov/M_oper)^(3/4)*(M_rob/(M_rob+M_oper))^(7/4);
%     if ~isreal([v])
%         fprintf('HIC imaginary\n ')
%     end
    HIC = abs(v)^(5/2)*beta;
end