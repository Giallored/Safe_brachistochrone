function v_safe = get_v_from_HIC(HIC_max,M_rob, M_oper, K_cov)
    beta = 2*(2/pi)^(3/2)*(K_cov/M_oper)^(3/4)*(M_rob/(M_rob+M_oper))^(7/4);
    v_safe = (HIC_max/beta)^(2/5);
end