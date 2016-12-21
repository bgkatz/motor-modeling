

%%% Load Motor Configuration %%%
motorConfig = 'ME_5208_274kv';
run(strcat('Motor Configs\', motorConfig));

%%% Transforms %%%

%%% Power-invariant form %%%
% dq0 = @(theta) sqrt(2/3)*[cos(theta), cos(theta - 2*pi/3), cos(theta+2*pi/3);
%     sin(theta), sin(theta - 2*pi/3), sin(theta + 2*pi/3);
%     1/sqrt(2), 1/sqrt(2), 1/sqrt(2)];

%abc = @(theta) inv(dq0(theta))
%%% Not your canonical transform, but it fits my assumptions %%%
abc = @(theta) sqrt(2/3)*[cos(-theta), sin(-theta), 1/sqrt(2);
    cos((2*pi/3)-theta), sin((2*pi/3)-theta), 1/sqrt(2);
    cos((-2*pi/3)-theta), sin((-2*pi/3)-theta), 1/(sqrt(2))];

dq0 = @(theta) abc(theta)';%inv(abc(theta));

%%% Inverter Properties %%%
f_switch = 40000;    %%Switching frequency
pwm_resolution = 10; %%Bits of PWM resolution
v_bus = 24;         %%Bus voltage

%%% Current Controller %%%

i_q_ref = sqrt(3/2)*40
i_d_ref = 0

i_dq0 = [0; 0];

r_s = r_a;
loop_dt = 1/f_switch;


ki_q = 1-exp(-r_s*loop_dt/l_q);
k_q = r_s*((2000*pi/(f_switch*4))/(1-exp(-r_s*loop_dt/l_q)));
ki_d = 1-exp(-r_s*loop_dt/l_d);
k_d = r_s*((2000*pi/(f_switch*4))/(1-exp(-r_s*loop_dt/l_d)));


q_int = 0;
d_int = 0;
q_int_max = sqrt(2)*v_bus;
d_int_max = sqrt(2)*v_bus;

dtc_uvw = [0; 0; 0];
v_d_cmd = 0;
v_q_cmd = 0;

%%% Mechanical Load %%%
J = .02; %%Kg-m^2
B  = 0; %%N-m*s/rad

%%% Initialize Dynamics Variables %%%
i = [0; 0; 0];
v = [0; 0; 0];
theta = 0;
thetadot = 1000;
thetadotdot = 0;
phase_shift = 0;

tfinal = .02;
dt = 1/(f_switch*(2^pwm_resolution));     %%Simulation time step, for n bits of PWM resolution
%dt = 1e-5;
t = 0:dt:tfinal;
wb_abc_rotor_old = [wb_r(theta, 0); wb_r(theta, 2*pi/3); wb_r(theta, -2*pi/3)];

timer_count = 0;

timer_step = (dt/(1/(2*f_switch)));
timer_dir = 1;

wb_old = Wb(theta, i);
l_old = L(theta);

thetadot_vec = zeros(length(t), 1);
v_vec = zeros(length(t), 3);
v_uvw_vec = zeros(length(t), 3);
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
cmd_vec  = zeros(length(t), 2);
int_vec = zeros(length(t), 2);
thetadot_mech_vec = zeros(length(t), 1);
torque_pm_vec = zeros(length(t), 1);
torque_rel_vec = zeros(length(t), 1);
tic

for j=1:length(t)
    time = t(j);
    
    %if(time > tfinal/2);
    %    i_q_ref = 0;
    %    i_d_ref = 0;
    %end
    
    %%% Controller %%%
    %%% PWM Timer %%%
    
    timer_count = timer_count + timer_step*timer_dir;
    if(timer_count > 1 || timer_count < 0)  %%up-down counting timer, center aligned pwm
        if(timer_count > 1)
            
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

            i_dq0 = dq0_transform*i_sample;

            %%% Controller %%%
            %%% Normal PI controller w/ feedforward decopuling and bemf compensation %%%
            i_q_error = i_q_ref - i_dq0(2);
            i_d_error = i_d_ref - i_dq0(1);
            
            v_q_coupling = l_d*i_dq0(1)*thetadot;
            v_d_coupling = -l_q*i_dq0(2)*thetadot;
            
            q_int = q_int + i_q_error*ki_q*k_q;
            d_int = d_int + i_d_error*ki_d*k_d;
            q_int = max(min(q_int, q_int_max), -q_int_max);
            d_int = max(min(d_int, d_int_max), -d_int_max);
            
            
            v_q_cmd = k_q*i_q_error + q_int + v_q_coupling;% + 2*v_dq0(2);
            v_d_cmd = k_d*i_d_error + d_int + v_d_coupling;% + 2*v_dq0(1);
            
            cmd_mag = norm([v_d_cmd, v_q_cmd]);
            
            ct = 0;
            
            %while((cmd_mag > (sqrt(3/2))*v_bus) & ct<300)
            %     v_q_cmd = .99*v_q_cmd;
            %     cmd_mag = norm([v_d_cmd, v_q_cmd]);
            %     ct = ct + 1;
            %end
            
            %if(cmd_mag > (sqrt(3/2)*v_bus))
            %   v_d_cmd = v_d_cmd*(sqrt(3/2)*v_bus/cmd_mag);
            %   v_q_cmd = v_q_cmd*(sqrt(3/2)*v_bus/cmd_mag);
            %end   
            
              %%% Calculate duty cycles %%%
            v_uvw_cmd = dq0_transform\[v_d_cmd; v_q_cmd; 0];
            v_offset = 0.5*(min(v_uvw_cmd) + max(v_uvw_cmd)); %%SVM
            v_uvw_cmd = v_uvw_cmd - v_offset;
            dtc_uvw = 0.5 + 0.5*((1/v_bus)*(v_uvw_cmd));
            dtc_uvw = max(min(dtc_uvw, 1), 0);  %% 0<=dtc<=1
            phase_shift = atan2(i_dq0(1), i_dq0(2));
            

        end
        timer_dir = timer_dir*-1;
    end
    
    
    %%% Terminal Voltages %%%
    v_uvw = v_bus*(dtc_uvw>timer_count);
    %v_abc = [0; 0; 0];
    
    %%% Phase Voltages from Terminal Voltages%%%
    if (strcmp(termination, 'delta'))
        v = [v_uvw(1)-v_uvw(2); v_uvw(2) - v_uvw(3); v_uvw(3) - v_uvw(1)];
    elseif (strcmp(termination, 'wye'))
        v = v_uvw;
    elseif (strcmp(termination, 'ind'))
        v = v_uvw;
    end
    
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
    %%% Phase Currents %%%
    i_dot = l\(v - v_bemf - R*i);
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
    
    
    thetadotdot = (torque)/J;
    thetadot = thetadot + thetadotdot*dt;
    thetadot_mech = thetadot;
    
    theta = theta + thetadot*dt;
%     wb = Wb(theta, i);
%     wb_dot = (wb - wb_old)*(1/dt);
%     v2 = R*i + wb_dot;
%     wb_old = wb;
   
    wb_abc_rotor_old = wb_abc_rotor;
    l_old = l;
    
    %%% Save Data %%%
    thetadot_vec(j) = thetadot;
    i_vec(j,:) = i';
    v_vec(j,:) = v'; 
    v_uvw_vec(j,:) = v_uvw';
    torque_vec(j) = torque;
    torque_abc_vec(j,:) = torque_abc';
    power_elec_abc_vec(j,:) = p_elec_abc';
    power_elec_vec(j) = p_elec;
    power_mech_abc_vec(j,:) = p_mech_abc';
    power_mech_vec(j) = p_mech;
    i_dq_vec(j,:) = [i_dq0(1); i_dq0(2)];
    v_bemf_vec(j,:) = v_bemf';
    phase_shift_vec(j) = phase_shift;
    cmd_vec(j,:) = [v_d_cmd; v_q_cmd];
    int_vec(j,:) = [d_int; q_int];
    thetadot_mech_vec(j) = thetadot_mech;
    torque_pm_vec(j) = torque_pm;
    torque_rel_vec(j) = torque_rel;

end
toc

%figure;plot(t, i_vec);
%figure;plot(t, v_bemf_vec);
%figure;plot(t, i_dq_vec); title('I D/Q');
figure;plot(t, torque_vec); title ('Torque');
hold all; plot(t, torque_pm_vec); plot(t, torque_rel_vec);
%figure;plot(t, thetadot_mech_vec); title('Theta dot');
%figure;plot(t, torque_abc_vec);

t_avg = mean(torque_vec);