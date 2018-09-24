function f = calc_force(theta, i, l_fun, fl_fun);
    delta = .001;
    ce1 = calc_coenergy(theta, i, l_fun, fl_fun, 100);
    ce2 = calc_coenergy(theta+delta, i, l_fun, fl_fun, 100);
    f = (ce1-ce2)/delta;