
%%%%%% Specify Motor Parameters %%%%%%
%%%%%% Edit these parameters %%%%%%
%%% Phase Flux Linkage, Wb %%%
k1 = 0.0024; %%harmonic coefficients
k3 = .000;
k5 = .00004;
k7 = 0;
k9 = 0.000; 

%%% Phase Resistances %%%
r_a = .015;
r_b = .015;
r_c = .015;

%%% Termination Type: wye, delta, ind %%%
termination = 'wye';

%%% Pole Pairs %%%
npp = 20;

%%% Inductances %%%
l_d = 15e-6;     %D-Axis Inductance
l_q = 15e-6;      %Q-Axis Inductance
l_m =  .00;    %Phase Mutual Inductance, assuming a constant for now

%%% Cogging %%%
t_cog = @(theta_r) .02*sin(6*theta_r);

%%%%% Automatic Setup %%%%%
%%%%% Don't change unless you know what you're doing %%%%%
%%% Resistance Matrix %%%
R = [r_a 0 0; 
    0 r_b 0; 
    0 0 r_c];

%%% pm flux linked by rotor at angle theta_r to phase at angle theta_p %%%
wb_r = @(theta_r, theta_p, iq) k1*cos(theta_p - theta_r) + k3*cos(3*(-theta_r +theta_p)) + k5*cos(5*(-theta_r+theta_p)) + k9*cos(9*(-theta_r) + theta_p);

dwb_r = @(theta_r, theta_p) k1*sin(theta_p - theta_r);

%%% Phase Self Inductance %%%dat
l_p = @(theta_r, theta_p) .5*(l_d - l_q)*(cos(2*(theta_p - theta_r)))+(l_d + l_q)/2;

%%% Derivative of Self Inductance wrt Rotor Angle %%%
dl_p = @(theta_r, theta_p) (l_d - l_q)*(sin(2*(theta_p - theta_r)));

%%% Inductance Matrix %%% 
L = @(theta_r, id, iq) [l_p(theta_r, 0), l_m, l_m; l_m, l_p(theta_r, 2*pi/3), l_m; l_m, l_m, l_p(theta_r, -2*pi/3)];

%%% Derivative of Inductance Matrix wrt Rotor Angle
dL = @(theta_r) [dl_p(theta_r, 0), 0, 0; 0, dl_p(theta_r, 2*pi/3), 0; 0, 0, dl_p(theta_r, -2*pi/3)];

%%% Flux Linkage Matrix %%%
Wb = @(theta_r, i, id, iq) [L(theta_r)]*i + [wb_r(theta_r, 0); wb_r(theta_r, 2*pi/3); wb_r(theta_r, -2*pi/3)];



