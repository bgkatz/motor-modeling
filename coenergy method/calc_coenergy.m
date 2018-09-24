function coenergy = calc_coenergy(theta, i, l_fun, fl_fun, n);
    i_vec = linspace(0, i, n);
    theta_vec = theta*ones(1, n);
    fl_vec = fl_fun(theta_vec, i_vec);
    l_vec = l_fun(theta_vec, i_vec);
    fl_total = fl_vec + i_vec.*l_vec;
    coenergy = cumtrapz(fl_total)*i/n;
    coenergy = coenergy(end);