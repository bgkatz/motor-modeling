function [ t, v_mag ] = motor_fun_current_mode(i_mag, i_phase, thetadot)

%%% Load Motor Configuration %%%
%%%%%% Specify Motor Parameters %%%%%%
%%%%%% Edit these parameters %%%%%%
%%% Flux Linkage, Wb %%%
k1 = 0.06; %%harmonic coefficients
k3 = .00;
k5 = -.00;
k7 = 0;
k9 = 0; %0.001;
%%% flux linked by rotor at angle theta_r to phase at angle theta_p %%%
wb_r = @(theta_r, theta_p) k1*cos(theta_p - theta_r) + k3*cos(3*(-theta_r) +theta_p) + k5*cos(5*(-theta_r)+theta_p) + k9*cos(9*(-theta_r) + theta_p);

%%% Phase Resistances %%%
r_a = .1;
r_b = .1;
r_c = .1;

%%% Termination Type: wye, delta, ind %%%
termination = 'wye';

%%% Pole Pairs %%%
npp = 3;

%%% Inductances %%%
l_d = .0008;     %D-Axis Inductance
l_q = .002;      %Q-Axis Inductance
l_m =  .00;    %Phase Mutual Inductance, assuming a constant for now

%%% Resistance Matrix %%%
R = [r_a 0 0; 
    0 r_b 0; 
    0 0 r_c];

%%% Phase Self Inductance %%%
l_p = @(theta_r, theta_p) .5*(l_d - l_q)*(cos(2*(theta_p - theta_r)))+(l_d + l_q)/2;

%%% Inductance Matrix %%% 
L = @(theta_r) [l_p(theta_r, 0), l_m, l_m; l_m, l_p(theta_r, 2*pi/3), l_m; l_m, l_m, l_p(theta_r, -2*pi/3)];

%%% Flux Linkage Matrix %%%
Wb = @(theta_r, i) [L(theta_r)]*i + [wb_r(theta_r, 0); wb_r(theta_r, 2*pi/3); wb_r(theta_r, -2*pi/3)];


%%% Transforms %%%
%%% Power-invariant form %%%
%%% Not your canonical transform, but it fits my assumptions %%%
abc = @(theta) sqrt(2/3)*[cos(-theta), sin(-theta), 1/sqrt(2);
    cos((2*pi/3)-theta), sin((2*pi/3)-theta), 1/sqrt(2);
    cos((-2*pi/3)-theta), sin((-2*pi/3)-theta), 1/(sqrt(2))];

dq0 = @(theta) abc(theta)';%inv(abc(theta));

%%% Inverter Properties %%%
f_switch = 10000;    %%Loop frequency
v_bus = 160;         %%Bus voltage

%%% Current Controller %%%

i_q = i_mag*sin(i_phase);
i_d = i_mag*cos(i_phase);

i_dq0 = [i_d; i_q; 0];

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
wb_abc_rotor_old = [wb_r(theta, 0); wb_r(theta, 2*pi/3); wb_r(theta, -2*pi/3)];

timer_step = (dt/(1/(2*f_switch)));
timer_dir = 1;

wb_old = Wb(theta, i);
l_old = L(theta);

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
    wb_abc_rotor = [wb_r(theta, 0); wb_r(theta, 2*pi/3); wb_r(theta, -2*pi/3)];
    wb_abc_rotor_dot = (wb_abc_rotor - wb_abc_rotor_old)*(1/dt);
    
    %%% Phase Inductance and derivative %%%
    l = L(theta);
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

%figure;plot(thetadot_vec, i_vec);
%figure;plot(t, v_bemf_vec);
%figure;plot(thetadot_vec, i_dq_vec); title('I D/Q');
%figure;plot(t, torque_vec); title ('Torque');
%hold all; plot(t, torque_pm_vec); plot(t, torque_rel_vec);
%figure;plot(t, thetadot_mech_vec); title('Theta dot');
%figure;plot(thetadot_mech_vec, torque_vec); title('Torque vs Speed');
%figure;plot(t, v_uvw_vec); title('UVW Voltages');
%figure;plot(t, v_dq0_vec); title('dq0 voltages');
%figure;plot(thetadot_mech_vec, power_mech_vec); title('Power vs Speed');
%figure;plotyy(t, phase_shift_vec, t, current_mag_vec);title('Current Phase/Mag');
%figure;plot(t, torque_abc_vec);

t = torque_vec(end);
v_mag = norm(v_dq0);

%toc

end

