function M_rob = get_M_rob(k,M_link,M_rot,gamma)
    M_rob = M_link + k/(k+gamma)*M_rot;
end