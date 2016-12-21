

%%% Load Motor Configuration %%%
motorConfig = 'ME_5208_274kv';
run(strcat('Motor Configs\', motorConfig));

%%% Transforms %%%
%%% Power-invariant form %%%
%%% Not your canonical transform, but it fits my assumptions %%%
abc = @(theta) sqrt(2/3)*[cos(-theta), sin(-theta), 1/sqrt(2);
    cos((2*pi/3)-theta), sin((2*pi/3)-theta), 1/sqrt(2);
    cos((-2*pi/3)-theta), sin((-2*pi/3)-theta), 1/(sqrt(2))];

dq0 = @(theta) abc(theta)';%inv(abc(theta));

%%% Inverter Properties %%%
f_switch = 20000;    %%Loop frequency
v_bus = 24;         %%Bus voltage

%%% Current Controller %%%

i_ref = sqrt(3/2)*40;
phase_ref = pi/2;
i_q_ref = i_ref*sin(phase_ref);
i_d_ref = i_ref*cos(phase_ref);

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
mag_int = 0;
phase_int = 0;
mag_int_max = sqrt(2)*v_bus;
phase_int_max = 4*pi;

dtc_abc = [0; 0; 0];
v_d_cmd = 0;
v_q_cmd = 0;

%%% Mechanical Load %%%
J = .00021; %%Kg-m^2
B  = 0.000007; %%N-m*s/rad

%%% Initialize Dynamics Variables %%%
i = [0; 0; 0];
v = [0; 0; 0];
v_bemf = v;
theta = 0;
thetadot = .1;
thetadotdot = 0;%.025;
phase_shift = 0;

tfinal = 8;
dt = 1/(f_switch);     %%Simulation time step
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
current_mag_vec = zeros(length(t), 1);
v_uvw_vec = zeros(length(t), 3);
tic

for j=1:length(t)
    time = t(j);
 
%     if(time > tfinal/2);
%         i_ref = 50;
%         %phase_ref = -pi;
%         %i_q_ref = -100
%         %i_d_ref = 2;
%         
%     end
    
    %%% Controller %%%    
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
    
    v_q_coupling = 2*l_d*i_dq0(1)*thetadot;
    v_d_coupling = -2*l_q*i_dq0(2)*thetadot;
    
    dq0_bemf = dq0_transform*v_bemf;
    %%% Controller %%%
   
    %%% Normal PI controller w/ feedforward decopuling and bemf feedforward %%%

    cmd_max = sqrt(3/2)*200;%*(thetadot<480) + (680-.2*thetadot)*(thetadot >=480);
    %i_ref = min(i_ref, cmd_max);

    %phase_ref = interp1(data(:,1), data(:,2), thetadot, 'linear', 'extrap');
    i_q_ref = i_ref*sin(phase_ref);
    i_d_ref = i_ref*cos(phase_ref);

    %i_q_ref = interp1(data(:,1), data(:,3), thetadot, 'linear');
    %i_d_ref = interp1(data(:,1), data(:,2), thetadot, 'linear');

    i_q_error = i_q_ref - i_dq0(2);
    i_d_error = i_d_ref - i_dq0(1);

    q_int = q_int + i_q_error*ki_q*k_q;
    d_int = d_int + i_d_error*ki_d*k_d;
    q_int = max(min(q_int, q_int_max), -q_int_max);
    d_int = max(min(d_int, d_int_max), -d_int_max);


    v_q_cmd = k_q*i_q_error + q_int;% + v_q_coupling + dq0_bemf(2);
    v_d_cmd = k_d*i_d_error + d_int;% + v_d_coupling + dq0_bemf(1);

    cmd_mag = norm([v_d_cmd, v_q_cmd]);
    %%% Limit voltage commands to not overmodulate %%%
    
    if(cmd_mag > (sqrt(2)*v_bus))
       v_d_cmd = v_d_cmd*(sqrt(2)*v_bus/cmd_mag);
       v_q_cmd = v_q_cmd*(sqrt(2)*v_bus/cmd_mag);
    end


    %end
    
    %%% Calculate actual inverter voltages %%%
    v_uvw_cmd = dq0_transform\[v_d_cmd; v_q_cmd; 0];
    v_offset = 0.5*(min(v_uvw_cmd) + max(v_uvw_cmd)); %%SVM
    v_uvw_cmd = v_uvw_cmd - v_offset;
    v_uvw_cmd = .5*v_bus + .5*v_uvw_cmd;
    v_uvw = max(min(v_uvw_cmd, v_bus), 0);
    %v_uvw = [0; 0; 0];
    phase_shift = atan2(i_dq0(2), i_dq0(1));
    

    
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
    p_mech_abc =  v_bemf.*i;
    p_mech = sum(p_mech_abc);
    
    p_pm = sum(v_pm.*i);
    p_rel = sum(v_rel.*i);
    
    %%% Phase Torques %%%
    torque_abc = p_mech_abc*(1/thetadot);
    
    
    %%% Total Torque %%%
    torque = sum(torque_abc);
    torque_pm = p_pm/thetadot;
    torque_rel= p_rel/thetadot;
    
    
    thetadotdot = (torque - B*thetadot)/J;
    thetadot = thetadot + thetadotdot*dt;
    thetadot_mech = thetadot;
    
    theta = theta + thetadot*dt;

   
    wb_abc_rotor_old = wb_abc_rotor;
    l_old = l;
    
    %%% Save Data %%%figue;
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
    v_n_vec(j) = v_n;
    %current_mag_vec(j) = sample_mag;
end
toc
uvw_diff_vec = [v_uvw_vec(:,1) - v_uvw_vec(:,2), v_uvw_vec(:,2) - v_uvw_vec(:,3), v_uvw_vec(:,3) - v_uvw_vec(:,1)];

%figure;plot(thetadot_vec, i_vec);
%figure;plot(t, v_bemf_vec);
figure;plot(t, i_dq_vec); title('I D/Q');
%figure;plot(t, torque_vec); title ('Torque');
%figure;plot(t, uvw_diff_vec); title('Line-To-Line Voltages @300 rad/s'); xlabel('Time (s)'); ylabel('Volts'); legend('U-V', 'V-W', 'W-U');

%hold all; plot(t, torque_pm_vec); plot(t, torque_rel_vec);
%figure;plot(t, thetadot_mech_vec); title('Theta dot');
figure;plot(thetadot_mech_vec, torque_vec); title('Torque vs Speed');
%figure;plot(t, v_uvw_vec); title('UVW Voltages');
%figure;plot(thetadot_mech_vec, power_mech_vec); title('Power vs Speed');
%figure;plotyy(t, phase_shift_vec, t, current_mag_vec);title('Current Phase/Mag');
%figure;plot(t, torque_abc_vec);

t_avg = mean(torque_vec);