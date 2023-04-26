init;
HIC_max = 200;
M_rob = M_rotor+M_link;

v_safe = get_v_from_HIC(HIC_max,M_rob,M_rob,K_cov);


%fprintf('\n v_safe = %d',v_safe)
T = optProblemRigid(M_rob,U_max,v_safe,q0,dq0,q1,dq1);


