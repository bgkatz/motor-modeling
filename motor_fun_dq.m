
function [ torque, v_mag ] = motor_fun_dq(i_mag, i_phase, thetadot)

%%% Motor Parameters
i_wb_ref = [0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200];
wb_lut = [0.0676, 0.0676, 0.0658, 0.0638, 0.0566, 0.0533, 0.0485, 0.0457, 0.0441, 0.0417, 0.0405];
wb_fun = @(iq) interp1(i_wb_ref, wb_lut, iq, 'pchip');

i_ref = [0; 40; 80; 120; 150; 200];
ld_lut = 1e-3*[.65;  .5655;.494;.26; .22;.20];
lq_lut = 1e-3*[1.465; 1.3754; 1.0740;0.550; 0.37; .31];
l_d_fun = @(id, iq) interp1(i_ref, ld_lut, abs(id), 'pchip');
l_q_fun = @(id, iq) interp1(i_ref, lq_lut, abs(iq), 'pchip');

r = .1;
ppairs = 3;

%%%

i_d = i_mag*cos(i_phase);
i_q = i_mag*sin(i_phase);

l_d = l_d_fun(i_d, i_q);
l_q = l_q_fun(i_d, i_q);
wb = wb_fun(i_q);

v_d = 1.5*r*i_d - 1.5*thetadot*l_q*i_q;
v_q = 1.5*r*i_q + 1.5*thetadot*l_d*i_d + 1.5*ppairs*thetadot*wb;

v_mag = norm([v_d, v_q]);
torque = ppairs*(3/2)*(wb+(l_d - l_q)*i_d)*i_q;
