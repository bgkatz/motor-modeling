%%% Fast version for optimization.  Fixes phase currents, returns torque and
%%% terminal voltage.

function [ t, v_mag ] = motor_fun_current_mode(i_mag, i_phase, thetadot)



%%%%%% Specify Motor Parameters %%%%%%
%%%%%% Edit these parameters %%%%%%
%%% Flux Linkage, Wb %%%
i_wb_ref = [0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200];
wb_lut = [0.0676, 0.0676, 0.0658, 0.0638, 0.0566, 0.0533, 0.0485, 0.0457, 0.0441, 0.0417, 0.0405];
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

l_m =  .00;    %Phase Mutual Inductance, assuming a constant for now

%%% Inductance Curves %%%

% i_ref = [0; 20; 40; 60; 80; 100; 120; 130; 150; 170; 200];
% ld_lut = 1e-3*[.65; .5785;  .5655; .533; .494; .351; .26; .2301; .20; .17; .14]+.4e-3; ;
% %lq_lut = 1e-3*[1.465; 1.3868; 1.3754; 1.2830;  1.0740; 0.8410; 0.550; 0.42; 0.32; .27; .25];
% lq_lut = 1e-3*[1.465; 1.465; 1.465; 1.465; 1.1571; 1.0052; .850; .7; .6; .5; .33]+.4e-3;;
% 
% %ld_lut = 1e-3*[.65; .5785;  .5655; .533; .494; .351; .26; .2301; .22; .21; .20];
% %lq_lut = 1e-3*[1.465; 1.3868; 1.3754; 1.2830;  1.0740; 0.8410; 0.550; 0.44; 0.37; .33; .31];
% 
% l_d = @(id, iq)  interp1(i_ref, ld_lut, norm([id, iq]), 'pchip');
% l_q = @(id, iq)  interp1(i_ref, lq_lut, norm([id, iq]), 'pchip');

i_ref = [0; 40; 80; 120; 150; 200];
ld_lut = 1e-3*[.65;  .5655;.494;.26; .22;.20];
lq_lut = 1e-3*[1.465; 1.3754; 1.0740;0.550; 0.37; .31];
l_d = @(id, iq) interp1(i_ref, ld_lut, abs(id), 'pchip');
l_q = @(id, iq) interp1(i_ref, lq_lut, abs(iq), 'pchip');


%%%%% Automatic Setup %%%%%
%%%%% Don't change unless you know what you're doing %%%%%
%%% Resistance Matrix %%%
R = [r_a 0 0; 
    0 r_b 0; 
    0 0 r_c];

%%% Phase Self Inductance %%%
%l_p = @(theta_r, theta_p, id, iq) .5*(l_d(id) - l_q(iq))*(cos(2*(-theta_r + theta_p)))+(l_d(id) + l_q(iq))/2;
l_p = @(theta_r, theta_p, id, iq) .5*(l_d(id, iq) - l_q(iq, iq))*(cos(2*(-theta_r + theta_p)))+(l_d(id, iq) + l_q(id, iq))/2;

%%% Inductance Matrix %%% 
L = @(theta_r, id, iq) [l_p(theta_r, 0, id, iq), l_m, l_m; l_m, l_p(theta_r, 2*pi/3, id, iq), l_m; l_m, l_m, l_p(theta_r, -2*pi/3, id, iq)];

%L_ll = @(theta_r) l_p(theta_r, 0)+l_p(theta_r, 2*pi/3);
%%% Flux Linkage Matrix %%%
Wb = @(theta_r, i, id, iq) [L(theta_r, id, iq)]*i + [wb_r(theta_r, 0, iq); wb_r(theta_r, 2*pi/3, iq); wb_r(theta_r, -2*pi/3, iq)];



%%% Transforms %%%
%%% Power-invariant form %%%
%%% Not your canonical transform, but it fits my assumptions %%%
abc = @(theta) [cos(-theta), sin(-theta), 1/sqrt(2);
    cos((2*pi/3)-theta), sin((2*pi/3)-theta), 1/sqrt(2);
    cos((-2*pi/3)-theta), sin((-2*pi/3)-theta), 1/(sqrt(2))];

dq0 = @(theta)  inv(abc(theta));

%%% Inverter Properties %%%
f_switch = 10000;    %%Loop frequency

%%% Current Controller %%%

iq = i_mag*sin(i_phase);
id = i_mag*cos(i_phase);

i_dq0 = [id; iq; 0];

%%% Initialize Dynamics Variables %%%

v = [0; 0; 0];
v_bemf = v;
theta = 0;
i = abc(theta)*i_dq0;
i_old = [0; 0; 0];
i_dot = [0; 0; 0];
thetadotdot = 0;
phase_shift = 0;

tfinal = .0002;
dt = 1/(f_switch);     %%Simulation time step
%dt = 1e-5;
t = 0:dt:tfinal;
wb_abc_rotor_old = [wb_r(theta, 0, iq); wb_r(theta, 2*pi/3, iq); wb_r(theta, -2*pi/3, iq)];

timer_step = (dt/(1/(2*f_switch)));
timer_dir = 1;

wb_old = Wb(theta, i, id, iq);
l_old = L(theta, id, iq);

thetadot_vec = zeros(length(t), 1);
v_vec = zeros(length(t), 3);
v_dq0_vec = zeros(length(t), 2);
i_vec = zeros(length(t), 3);
torque_vec = zeros(length(t), 1);
power_mech_vec = zeros(length(t), 1);
v_uvw_vec = zeros(length(t), 3);


%tic
for j=1:length(t)
    time = t(j);
 
    %%% Sample Current %%%
    
    if (strcmp(termination, 'delta'))
        i_sample = [i(1)-i(3); i(2) - i(1); i(3) - i(2)];
    elseif (strcmp(termination, 'wye'))
        i_sample = i;
    elseif (strcmp(termination, 'ind'))
        i_sample = i;
    end
    
    %%% Calculate Transform Matrix %%%
    %%dq0_transform = dq0(theta);
    abc_transform = abc(theta);
    dq0_transform = inv(abc_transform);
    
    i = abc_transform*i_dq0;
    i_dot = (i - i_old)*(1/dt);
    i_old = i;
    


   

    %%% Rotor Flux linked to each phase, and derivative %%
    wb_abc_rotor = [wb_r(theta, 0, iq); wb_r(theta, 2*pi/3, iq); wb_r(theta, -2*pi/3, iq)];
    wb_abc_rotor_dot = (wb_abc_rotor - wb_abc_rotor_old)*(1/dt);
    
    %%% Phase Inductance and derivative %%%
    
    l = L(theta, id, iq);
    l_dot = (l-l_old)*(1/dt);
    
    %%% Back-EMF %%%
    v_bemf = (l_dot*i + wb_abc_rotor_dot);
    
    
    %%% Phase Voltages %%%
     value = [l, [1;1;1]; [1 1 1 0]]*[i_dot; 0] + [v_bemf;0] + [R, [0; 0; 0]; [0 0 0 0]]*[i; 0];
     v_uvw = value(1:3);
     v_dq0 = dq0_transform*v_uvw;
     v_dq0 = v_dq0(1:2);

    %%% Mechanical Power %%%
    p_mech_abc =  v_bemf.*i;
    p_mech = sum(p_mech_abc);
    
    %%% Phase Torques %%%
    torque_abc = p_mech_abc*(1/thetadot);
    
    %%% Total Torque %%%
    torque = sum(torque_abc);

    thetadot = thetadot + thetadotdot*dt;
    thetadot_mech = thetadot;
    
    theta = theta + thetadot*dt;

   
    wb_abc_rotor_old = wb_abc_rotor;
    l_old = l;
    
    %%% Save Data %%%
    thetadot_vec(j) = thetadot;
    i_vec(j,:) = i';
    v_vec(j,:) = v'; 
    v_uvw_vec(j,:) = v_uvw';
    v_dq0_vec(j, :) = v_dq0';
    torque_vec(j) = torque;
    power_mech_vec(j) = p_mech;
    %current_mag_vec(j) = sample_mag;
end
%toc


t = torque_vec(end);
v_mag = norm(v_dq0);

%toc

end

