function M_rob = get_M_rob(k,M_link,M_rot,x,gamma)
    M_rob = M_link + k/(k+gamma)*M_rot;
    delta_x = abs(x(1)-x(2));
    M_rob = M_link + (2*k*delta_x/(2*k*delta_x+1)*M_rot);
end