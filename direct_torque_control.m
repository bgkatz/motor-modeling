%%% Playing with direct torque control %%%


clear all;
format compact;
format long g;

%%% Load Motor Configuration %%%
motorConfig = 'EX_8';
run(strcat('Motor Configs\', motorConfig));

%%% Transforms %%%

%%% Power-invariant form %%%
% dq0 = @(theta) sqrt(2/3)*[cos(theta), cos(theta - 2*pi/3), cos(theta+2*pi/3);
%     sin(theta), sin(theta - 2*pi/3), sin(theta + 2*pi/3);
%     1/sqrt(2), 1/sqrt(2), 1/sqrt(2)];

%abc = @(theta) inv(dq0(theta))
%%% Not your canonical transform, but it fits my assumptions %%%
%%% 
abc = @(theta) [cos(-theta), sin(-theta), 1/sqrt(2);
    cos((2*pi/3)-theta), sin((2*pi/3)-theta), 1/sqrt(2);
    cos((-2*pi/3)-theta), sin((-2*pi/3)-theta), 1/(sqrt(2))];

dq0 = @(theta) (2/3)*abc(theta)'; %= inv(abc)

%%% Inverter Properties %%%
v_bus = 24;         %%Bus voltage
i_max = 40;   %%Current at ADC ful scale
adc_res = 12; %%Bits of ADC Resolution
i_noise = 1;  %Bits of current sensor noise

%%% Controller %%%
spl = 4;            %%simulation steps per loop
f_switch = 40000;
f_sim = spl*f_switch;
i_dq0 = [0; 0];
r_s = r_a;
loop_dt = 1/f_switch;

v_uvw = [0; 0; 0];
v_uvw_cmd = [0; 0; 0];

switch_state = [0; 0; 0];


torque_ref = 1;
torque_est = [0];
wb_stator_est = [0; 0; 0];

%%% Mechanical Load %%%
J = .05; %%Kg-m^2
B  = 0; %%N-m*s/rad
tau_ext = 0;
%%% Initialize Dynamics Variables %%%
i = [0; 0; 0];
i_dot = [0; 0; 0];
i_sample = [0; 0; 0];
i_sample_old = i_sample;
v = [0; 0; 0];
theta = pi/6;
thetadot = .01;
thetadotdot = 0;
phase_shift = 0;

tfinal = .002;
dt = 1/(f_sim);     %%Simulation time step
%dt = 1e-5;
t = 0:dt:tfinal;

wb_abc_rotor_old = [wb_r(theta, 0); wb_r(theta, 2*pi/3); wb_r(theta, -2*pi/3)];

timer_count = 0;

timer_step = (dt/(1/(2*f_switch)));
timer_dir = 1;

wb_old = Wb(theta, i);
l_old = L(theta);

dtc_uvw = [0; 0; 0];
switching_event = 0;
t_last = -dt;

thetadot_vec = zeros(length(t), 1);
v_vec = zeros(length(t), 3);
v_uvw_vec = zeros(length(t), 3);
v_uvw_cmd_vec = zeros(length(t), 3);
i_vec = zeros(length(t), 3);
i_sample_vec = zeros(length(t), 3);
torque_abc_vec = zeros(length(t), 3);
torque_vec = zeros(length(t), 1);
power_elec_abc_vec = zeros(length(t), 3);
power_elec_vec = zeros(length(t), 1);
power_mech_abc_vec = zeros(length(t), 3);
power_mech_vec = zeros(length(t), 1);
i_dq_vec = zeros(length(t), 2);
v_bemf_vec = zeros(length(t), 3);
cmd_vec  = zeros(length(t), 2);
int_vec = zeros(length(t), 2);
thetadot_mech_vec = zeros(length(t), 1);
torque_pm_vec = zeros(length(t), 1);
torque_rel_vec = zeros(length(t), 1);
switching_vec = zeros(length(t), 3);
wb_est_vec = zeros(length(t), 3);
tic


for j=1:length(t)
    time = t(j);
    
    if(time > tfinal/2);

    end
    
    %%% Controller %%%
    %%% PWM Timer %%%
    timer_count = timer_count + 1;
     if(timer_count > spl)
            
            %%% Sample Current %%%
            if (strcmp(termination, 'delta'))
                i_sample = [i(1)-i(3); i(2) - i(1); i(3) - i(2)]; 
            elseif (strcmp(termination, 'wye'))
                i_sample = i;
            elseif (strcmp(termination, 'ind'))
                i_sample = i;
            end
            
            %%% Current sensor quantization and noise %%%
            i_sample = (2*i_max/(2^adc_res))*(floor((2^adc_res)*(1/(2*i_max))*i_sample) + randi([-2^(i_noise-1),2^(i_noise-1)],[3,1]));
            
           
            %%% Current Transforms %%%
            abc_transform = abc(theta);
            dq0_transform = inv(abc_transform);

            i_dq0 = dq0_transform*i_sample;
            
            l = L(theta);
            wb_stator_est  = wb_stator_est + l*v;
            
            timer_count = 0;
        end
    
    
    
    v_uvw = v_bus*switch_state;

    %%% Rotor Flux linked to each phase, and derivative %%
    wb_abc_rotor = [wb_r(theta, 0); wb_r(theta, 2*pi/3); wb_r(theta, -2*pi/3)];
    wb_abc_rotor_dot = (wb_abc_rotor - wb_abc_rotor_old)*(1/dt);
    wb_abc_rotor_old = wb_abc_rotor;

    %%% Phase Inductance and derivative %%%
    l = L(theta);
    l_dot = (l-l_old)*(1/dt);
    l_old = l;

      %%% Back-EMF %%%
    v_bemf = (l_dot*i + wb_abc_rotor_dot);

    v_pm = wb_abc_rotor_dot;
    v_rel = l_dot*i;

    %%% Phase Voltages from Terminal Voltages, Solve di/dt%%%
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


    %%% Phase Currents %%%
    %i_dot = l\(v - v_bemf - R*i);
    i = i + i_dot*dt;
    
    %%% Terminal Power %%%
    p_elec_abc = v.*i;
    p_elec = sum(p_elec_abc);
    
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
    
    
    thetadotdot = (torque - tau_ext)/J;
    thetadot = thetadot + thetadotdot*dt;
    thetadot_mech = thetadot;
    
    theta = theta + thetadot*dt;
%     wb = Wb(theta, i);
%     wb_dot = (wb - wb_old)*(1/dt);
%     v2 = R*i + wb_dot;
%     wb_old = wb;
   
    
    
    
    %%% Save Data %%%
    thetadot_vec(j) = thetadot;
    i_vec(j,:) = i';
    i_sample_vec(j,:) = i_sample';
    v_vec(j,:) = v'; 
    v_uvw_vec(j,:) = v_uvw';
    v_uvw_cmd_vec(j,:) = v_uvw_cmd';
    torque_vec(j) = torque;
    torque_abc_vec(j,:) = torque_abc';
    power_elec_abc_vec(j,:) = p_elec_abc';
    power_elec_vec(j) = p_elec;
    power_mech_abc_vec(j,:) = p_mech_abc';
    power_mech_vec(j) = p_mech;
    i_dq_vec(j,:) = [i_dq0(1); i_dq0(2)];
    v_bemf_vec(j,:) = v_bemf';
    thetadot_mech_vec(j) = thetadot_mech;
    torque_pm_vec(j) = torque_pm;
    torque_rel_vec(j) = torque_rel;
    switching_vec(j,:) = switching_event;
    wb_est_vec(j,:) = wb_stator_est;

end
toc

%figure;plot(t, i_vec); hold all; plot(t, i_sample_vec);
figure;plot(t, i_dq_vec);
figure;plot(wb_est_vec);
%figure;plot(t, v_bemf_vec);
%figure;plot(t, i_dq_vec); title('I D/Q');
%figure;plot(t, torque_vec); title ('Torque');
%hold all; plot(t, torque_pm_vec); plot(t, torque_rel_vec);
%figure;plot(t, thetadot_mech_vec); title('Theta dot');
%figure;plot(t, torque_abc_vec);

%t_avg = mean(torque_vec);