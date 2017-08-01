%%% Fast version for optimization.  Fixes phase currents, returns torque and
%%% terminal voltage.

function [ t_pm, t_rel ] = motor_fun_current_mode(i_d, i_q)

%%% Load Motor Configuration %%%
motorConfig = 'Altermotter';
run(strcat('Motor Configs\', motorConfig));

%%% Transforms %%%
%%% Power-invariant form %%%
%%% Not your canonical transform, but it fits my assumptions %%%
abc = @(theta) [cos(theta), -sin(theta), 1/2;
    cos((-2*pi/3)+theta), -sin((-2*pi/3)+theta), 1/2;
    cos((2*pi/3)+theta), -sin((2*pi/3)+theta), 1/2];

dq0 = @(theta) inv(abc(theta));

%%% Inverter Properties %%%
f_switch = 10000;    %%Loop frequency

%%% Current Controller %%%



i_dq0 = [i_d; i_q; 0];

%%% Initialize Dynamics Variables %%%

v = [0; 0; 0];
v_bemf = v;
theta = 0;
i = abc(theta)*i_dq0;
i_old = [0; 0; 0];
i_dot = [0; 0; 0];
thetadot = .01;
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
t_pm_vec = zeros(length(t), 1);
t_rel_vec = zeros(length(t), 1);

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
        v_pm = wb_abc_rotor_dot;
    v_rel = l_dot*i;
    
    %%% Phase Voltages %%%
     value = [l, [1;1;1]; [1 1 1 0]]*[i_dot; 0] + [v_bemf;0] + [R, [0; 0; 0]; [0 0 0 0]]*[i; 0];
     v_uvw = value(1:3);
     v_dq0 = dq0_transform*v_uvw;
     v_dq0 = v_dq0(1:2);


    %%% Mechanical Power %%%
    p_mech_abc = v_bemf.*i;
    p_mech = sum(p_mech_abc);
    
    p_pm = sum(v_pm.*i);
    p_rel = sum(v_rel.*i);
    
    %%% Phase Torques %%%
    torque_abc = p_mech_abc*(1/thetadot);
    
    
    %%% Total Torque %%%
    torque = sum(torque_abc);
    torque_pm = p_pm/thetadot;
    torque_rel= p_rel/thetadot;

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
    t_pm_vec(j) = torque_pm;
    t_rel_vec(j) = torque_rel;
    %current_mag_vec(j) = sample_mag;
end
%toc


t_pm = t_pm_vec(end);
t_rel = t_rel_vec(end);

%toc

end

