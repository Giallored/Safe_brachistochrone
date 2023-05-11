function v_safe = get_v_safe(uk,M_rob,M_oper,K_cov, HIC_max)

gamma =300;

v_safe = get_v_from_HIC(HIC_max, M_rob,M_oper,K_cov);

end
