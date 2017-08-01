
%%%%%% Specify Motor Parameters %%%%%%
%%%%%% Edit these parameters %%%%%%
%%% Flux Linkage, Wb %%%

i_wb_ref = [0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200];
wb_lut = [0.0676, 0.0676, 0.0658, 0.0638, 0.0566, 0.0533, 0.0485, 0.0457, 0.0441, 0.0417, 0.0405];
wb_nom = wb_lut(1);
k1 = @(iq) interp1(i_wb_ref, wb_lut, iq, 'pchip');
%%% pm flux linked by rotor at angle theta_r to phase at angle theta_p %%%
wb_r = @(theta_r, theta_p, iq) k1(iq)*cos(theta_p - theta_r);

%%% Phase Resistances %%%
r_a = .1;
r_b = .1;
r_c = .1;

%%% Termination Type: wye, delta, ind %%%
termination = 'wye';

%%% Pole Pairs %%%
npp = 3;

%%% Nominal Inductances %%%
l_d_nom = .00065;     %D-Axis Inductance
l_q_nom = .001465;      %Q-Axis Inductance
l_m =  0;%.1*l_q_nom;    %Phase Mutual Inductance, assuming a constant for now

%%% Inductance Curves %%%
%i_ref = [0; 20; 40; 60; 80; 100; 120; 130; 150; 170; 200];
%ld_lut = 1e-3*[.65; .5785;  .5655; .533; .494; .351; .26; .2301; .22; .21; .20];
%lq_lut = 1e-3*[1.465; 1.3868; 1.3754; 1.2830;  1.0740; 0.8410; 0.550; 0.44; 0.37; .33; .31];

i_ref = [0; 40; 80; 120; 150; 200];
ld_lut = 1e-3*[.65;.5655;.494;.35; .3;.20];
lq_lut = 1e-3*[1.465; 1.3754; 1.0740;0.550; 0.37; .31];
l_d = @(id) interp1(i_ref, ld_lut, abs(id), 'pchip');
l_q = @(iq) interp1(i_ref, lq_lut, abs(iq), 'pchip');



%%%%% Automatic Setup %%%%%
%%%%% Don't change unless you know what you're doing %%%%%
%%% Resistance Matrix %%%
R = [r_a 0 0; 
    0 r_b 0; 
    0 0 r_c];

%%% Phase Self Inductance %%%
l_p = @(theta_r, theta_p, id, iq) .5*(l_d(id) - l_q(iq))*(cos(2*(-theta_r + theta_p)))+(l_d(id) + l_q(iq))/2;

%%% Inductance Matrix %%% 
L = @(theta_r, id, iq) [l_p(theta_r, 0, id, iq), l_m, l_m; l_m, l_p(theta_r, 2*pi/3, id, iq), l_m; l_m, l_m, l_p(theta_r, -2*pi/3, id, iq)];

%L_ll = @(theta_r) l_p(theta_r, 0)+l_p(theta_r, 2*pi/3);
%%% Flux Linkage Matrix %%%
Wb = @(theta_r, i, id, iq) [L(theta_r, id, iq)]*i + [wb_r(theta_r, 0, iq); wb_r(theta_r, 2*pi/3, iq); wb_r(theta_r, -2*pi/3, iq)];

