function v_safe = get_v_safe(uk,M_link, M_rot,M_oper,K_cov, HIC_max)

gamma =1;
M_rob = M_link + uk/(uk+gamma)*M_rot;

v_safe = get_v_from_HIC(HIC_max, M_rob,M_oper,K_cov);

end
