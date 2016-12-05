function [ torque_avg, i_d, i_q] = motor_fun( v_mag, v_phase, tdot)
%clear all
%tic

% v_mag = sqrt(2)*160;
% v_phase = pi/2;
% tdot = 1000;

%%% Load Motor Configuration %%%
motorConfig = 'Altermotter';
run(strcat('Motor Configs\', motorConfig));

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
loop_dt = 1/f_switch;

v_d_cmd = v_mag*cos(v_phase);
v_q_cmd = v_mag*sin(v_phase);

%%% Initialize Dynamics Variables %%%
i = [0; 0; 0];    
v = [0; 0; 0];
theta = 0;
thetadot = tdot;
thetadotdot = 0;
phase_shift = 0;

tfinal = .09;
dt = 1/(f_switch);     %%Simulation time step
%dt = 1e-5;
t = 0:dt:tfinal;
wb_abc_rotor_old = [wb_r(theta, 0); wb_r(theta, 2*pi/3); wb_r(theta, -2*pi/3)];

wb_old = Wb(theta, i);
l_old = L(theta);

thetadot_vec = zeros(length(t), 1);
v_vec = zeros(length(t), 3);
i_vec = zeros(length(t), 3);
torque_abc_vec = zeros(length(t), 3);
torque_vec = zeros(length(t), 1);
power_elec_abc_vec = zeros(length(t), 3);
power_elec_vec = zeros(length(t), 1);
power_mech_abc_vec = zeros(length(t), 3);
power_mech_vec = zeros(length(t), 1);
i_dq_vec = zeros(length(t), 2);
v_bemf_vec = zeros(length(t), 3);
phase_shift_vec = zeros(length(t), 1);
thetadot_mech_vec = zeros(length(t), 1);
current_mag_vec = zeros(length(t), 1);
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
    dq0_transform = abc_transform';
    
    i_dq0 = dq0_transform*i_sample;
    current_phase = atan2(i_dq0(2), i_dq0(1));
    i_mag = norm(i_dq0);
    %%% Calculate actual inverter voltages %%%
    v_uvw_cmd = dq0_transform\[v_d_cmd; v_q_cmd; 0];
    v_offset = 0.5*(min(v_uvw_cmd) + max(v_uvw_cmd)); %%SVM
    v_uvw_cmd = v_uvw_cmd - v_offset;
    v_uvw_cmd = .5*v_bus + .5*v_uvw_cmd;
    v_uvw = max(min(v_uvw_cmd, v_bus), 0);
   
    %%% Rotor Flux linked to each phase, and derivative %%
    wb_abc_rotor = [wb_r(theta, 0); wb_r(theta, 2*pi/3); wb_r(theta, -2*pi/3)];
    wb_abc_rotor_dot = (wb_abc_rotor - wb_abc_rotor_old)*(1/dt);
    
    %%% Phase Inductance and derivative %%%
    l = L(theta);
    l_dot = (l-l_old)*(1/dt);
    
    %%% Back-EMF %%%
    v_pm = wb_abc_rotor_dot;    %PM Back emf
    v_rel = l_dot*i;            %Reluctance Back emf
    
    v_bemf = (v_rel + v_pm);
    
    %%% Phase Currents %%%
  
    if (strcmp(termination, 'delta'))
        v = [v_uvw(1)-v_uvw(2); v_uvw(2) - v_uvw(3); v_uvw(3) - v_uvw(1)];
        i_dot = l\(v - v_bemf - R*i);
    elseif (strcmp(termination, 'wye'))
        value = [l, [1;1;1]; [1 1 1 0]]\([v_uvw;0] - [v_bemf;0] - [R, [0; 0; 0]; [0 0 0 0]]*[i; 0]);
        i_dot = value(1:3);
        v_n = value(4);
        v = v_uvw - [v_n; v_n; v_n];
    elseif (strcmp(termination, 'ind'))
        v = v_uvw;
        i_dot = l\(v - v_bemf - R*i);
    end
    
    i = i + i_dot*dt;
    
    %%% Terminal Power %%%
    p_elec_abc = v_uvw.*i;
    p_elec = sum(p_elec_abc);
    
    %%% Mechanical Power %%%
    p_mech_abc = v_bemf.*i;%(v_uvw).*i - (R*i).*i;
    p_mech = sum(p_mech_abc);
    
    p_pm = sum(v_pm.*i);
    p_rel = sum(v_rel.*i);
    
    %%% Phase Torques %%%
    torque_abc = p_mech_abc*(1/thetadot);
    
    
    %%% Total Torque %%%
    torque = sum(torque_abc);
    torque_pm = p_pm/thetadot;
    torque_rel= p_rel/thetadot;
    
    
    thetadotdot = 0;%(torque - B*thetadot)/J;
    %thetadot = thetadot;
    thetadot_mech = thetadot;
    
    theta = theta + thetadot*dt;

   
    wb_abc_rotor_old = wb_abc_rotor;
    l_old = l;
    
    %%% Save Data %%%
    thetadot_vec(j) = thetadot;
    i_vec(j,:) = i';
    v_vec(j,:) = v'; 
    v_uvw_vec(j,:) = v_uvw';
    torque_vec(j) = torque;
    torque_abc_vec(j,:) = torque_abc';
    power_mech_abc_vec(j,:) = p_mech_abc';
    power_mech_vec(j) = p_mech;
    i_dq_vec(j,:) = [i_dq0(1); i_dq0(2)];
    thetadot_mech_vec(j) = thetadot_mech;
    current_mag_vec(j) = i_mag;
    phase_shift_vec(j) = current_phase;
end
%toc

%figure;plot(t, i_vec);
%figure;plot(t, v_bemf_vec);
figure;plot(t, i_dq_vec); title('I D/Q');
figure; plot(t, torque_vec); title ('Torque');
%figure;plot(t, thetadot_mech_vec); title('Theta dot');
%figure;plot(t, torque_abc_vec);
%figure;plot(t, v_uvw_vec);
%figure;plot(int_vec);

torque_avg = torque_vec(end);
i_mag = norm(i_dq0);
i_d = i_dq0(1);
i_q = i_dq0(2);
% v_bemf
% torque_abc
% current_phase
%i_dq0
%toc

end

