%%% This version has ny switching dynamics, so can be much faster.  Usefull
%%% for long time-scale modeling, or if switching speed effects don't
%%% matter

%%% Load Motor Configuration %%%
motorConfig = 'Altermotter';
run(strcat('Motor Configs\', motorConfig));

%%% Transforms %%%
%%% Power-invariant form %%%
%%% Not your canonical transform, but it fits my assumptions %%%
abc = @(theta) [cos(-theta), sin(-theta), 1/sqrt(2);
    cos((2*pi/3)-theta), sin((2*pi/3)-theta), 1/sqrt(2);
    cos((-2*pi/3)-theta), sin((-2*pi/3)-theta), 1/(sqrt(2))];

dq0 = @(theta) inv(abc(theta)); %= inv(abc)

%%% Inverter Properties %%%
spl = 5;            %%simulation steps per loop
f_switch = 10000;       %% maximum switching frequency
f_loop = 30000;    %%Loop frequency
f_sim = spl*f_loop;
v_bus = 160;         %%Bus voltage
switch_delay = 13e-6;

%%% Current Controller %%%

%i_ref = 200*sqrt(2);
%phase_ref = 2;
%i_q_ref = i_ref*sin(phase_ref);
%i_d_ref = i_ref*cos(phase_ref);

%i_q_ref = 56.59;
%i_d_ref = -101.5
pmt_old = 0;

i_dq0 = [0; 0];

r_s = r_a;
loop_dt = 1/f_loop;


ki_q = 1-exp(-r_s*loop_dt/(l_q_nom));
k_q = r_s*((100*pi/(f_switch))/(1-exp(-r_s*loop_dt/l_q_nom)));
ki_d = 1-exp(-r_s*loop_dt/(l_d_nom));
k_d = r_s*((100*pi/(f_switch))/(1-exp(-r_s*loop_dt/l_d_nom)));


% k_q = 2.5;
% ki_q = .01;
% k_d = 2;
% ki_d = .025;

i_q_ref = 0;
i_d_ref = 0;
        
q_int = 0;
d_int = 0;
q_int_max = 0;v_bus;
d_int_max = 0;v_bus;
mag_int = 0;
mag_int_max = v_bus;
i_q_ref_filt = 0;
i_d_ref_filt = 0;
theta_s = 0;

dtc_abc = [0; 0; 0];
v_d_cmd = 0;
v_q_cmd = 0;
v_d_ff = 0;
v_q_ff = 0;
v_dq0_actual = [0; 0; 0];
switch_state = [0 0 0];
time_since_switch = 0;
v_uvw = [0; 0; 0];

wb_stator_est = [0; 0; 0];
%%% Mechanical Load %%%
J = .005; %%Kg-m^2
B  = 0.05; %%N-m*s/rad

%%% Initialize Conditions %%%
i = [0; 0; 0];
v = [0; 0; 0];
v_bemf = v;
theta = 0.1;
thetadot = 500;
thetadotdot = 0;%.025;
phase_shift = 0;

tfinal = .05;
dt = 1/(f_sim);     %%Simulation time step
%dt = 1e-5;
t = 0:dt:tfinal;
wb_abc_rotor_old = [wb_r(theta, 0, i_dq0(2)); wb_r(theta, 2*pi/3, i_dq0(2)); wb_r(theta, -2*pi/3, i_dq0(2))];

count = spl;

timer_step = (dt/(1/(2*f_loop)));
timer_dir = 1;

wb_old = Wb(theta, i, i_dq0(1), i_dq0(2));
l_old = L(theta, i_dq0(1), i_dq0(2));

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
i_dq_ref_vec = zeros(length(t), 2);
v_bemf_vec = zeros(length(t), 3);
phase_shift_vec = zeros(length(t), 1);
cmd_vec  = zeros(length(t), 2);
int_vec = zeros(length(t), 2);
thetadot_mech_vec = zeros(length(t), 1);
torque_pm_vec = zeros(length(t), 1);
torque_rel_vec = zeros(length(t), 1);
current_mag_vec = zeros(length(t), 1);
v_uvw_vec = zeros(length(t), 3);
v_uvw_cmd_vec = zeros(length(t), 3);
wb_est_vec = zeros(length(t), 3);
t_est_vec = zeros(length(t), 3);
l_vec = zeros(length(t), 3);
v_ff_vec = zeros(length(t), 2);
switch_choice_vec = zeros(length(t), 1);
theta_vec = zeros(length(t), 1);
tic

%sv = linspace(0, pmt_lut.speed(end), length(pmt_lut.pmt(1,:)));
%pv = linspace(0, 1, length(pmt_lut.pmt(:,1)));
        
for j=1:length(t)
    time = t(j);
    
    if(time > tfinal/6);
        %tau_ext = 1000;
        i_q_ref = 19;
        i_d_ref = -5;
    end
    if(time>3.5*tfinal/6);
        i_q_ref = 19;
        i_d_ref = -50;
    end
    
    %%% Sample Current %%%
    if (strcmp(termination, 'delta'))
            i_sample = [i(1)-i(3); i(2) - i(1); i(3) - i(2)];
        elseif (strcmp(termination, 'wye'))
            i_sample = i + 3*(rand(3, 1)-.5);
        elseif (strcmp(termination, 'ind'))
            i_sample = i;
        end

        %%% Calculate Transform Matrix %%%
        %%dq0_transform = dq0(theta);
        abc_transform = abc(theta);
        dq0_transform = inv(abc_transform);

        i_dq0 = dq0_transform*i_sample;
        
    %control loop
    if(count == spl)
        %v_q_coupling = 2*l_d*i_dq0(1)*thetadot;
        %v_d_coupling = -2*l_q*i_dq0(2)*thetadot;

        dq0_bemf = dq0_transform*v_bemf; 
        %%% Controller %%%

        %%% Normal PI controller w/ feedforward decopuling and bemf feedforward %%%

        %pmt = 1;
        %pmt_ind = floor(pmt*(length(pmt_lut.pmt(:,1))));
        %speed_ind = floor(length(pmt_lut.speed(1,:))*(thetadot/(pmt_lut.speed(end)))) + 1;
        
        

        
        i_q_ref_filt = .02*i_q_ref + .98*i_q_ref_filt;
        i_d_ref_filt = .02*i_d_ref + .98*i_d_ref_filt;
        
        i_q_error = i_q_ref_filt - i_dq0(2);
        i_d_error = i_d_ref_filt - i_dq0(1);

        q_int = q_int + i_q_error*ki_q*k_q;
        d_int = d_int + i_d_error*ki_d*k_d;

        q_int = max(min(q_int, q_int_max), -q_int_max);
        d_int = max(min(d_int, d_int_max), -d_int_max);
         
        int_mag = norm([q_int, d_int]);

        
            
         %v_d_ff = 2*i_d_ref*r_a;
         %v_q_ff = 2*(i_q_ref*r_a + sqrt(3/2)*k1*thetadot);

        v_q_cmd = k_q*i_q_error + q_int;% +sqrt(3)*r_a*i_q_ref + sqrt(3/2)*thetadot*l_d_nom*i_dq0(1) + sqrt(3)*thetadot*wb_nom;%k_q*i_q_error + q_int;% + v_q_ff;% + v_q_coupling;% + dq0_bemf(2);
        v_d_cmd = k_d*i_d_error + d_int;% + sqrt(3)*r_a*i_d_ref - sqrt(3/2)*thetadot*l_q_nom*i_dq0(2);%k_d*i_d_error + d_int;% + v_q_ff;% + v_d_coupling;% + dq0_bemf(1);
        
        
        
        %v_q_cmd = v_q_ff;
        %v_d_cmd = v_d_ff;
        %%% Limit voltage commands to not overmodulate %%%

        cmd_mag = norm([v_d_cmd, v_q_cmd]);
        %%% Limit voltage commands to not overmodulate %%%
        if(cmd_mag > (v_bus))
           v_d_cmd = v_d_cmd*(v_bus/cmd_mag);
           v_q_cmd = v_q_cmd*(v_bus/cmd_mag);
        end
        
%         tol = 5;
%             i_q_error = (i_q_ref_filt - i_dq0(2));
%             i_d_error = (i_d_ref_filt - i_dq0(1));
%             v_q_ff = 1.5*r_a*i_dq0(2) + 1.5*thetadot*l_d(i_dq0(1))*i_dq0(1) + 1.5*thetadot*wb_nom;
%             v_d_ff = 1.5*r_a*i_dq0(1) - 1.5*thetadot*l_q(i_dq0(2))*i_dq0(2);% + 1.5*.15e-3*;
%             v_dynamics = [cos(-theta), -sin(-theta); sin(-theta), cos(-theta)]*[v_d_ff; v_q_ff];
%             v_q_vec =  i_q_error;% + v_q_ff;% + sqrt(3)*r_a*i_dq0(2);% + sqrt(3)*thetadot*l_d_nom*i_dq0(1) + sqrt(3)*thetadot*wb_nom;
%             v_d_vec =  i_d_error;% + v_d_ff;% + sqrt(3)*r_a*i_dq0(1);% - sqrt(3)*thetadot*l_q_nom*i_dq0(2);
%             
%             
%             %if(norm([i_q_error, i_d_error]) > tol)
%             %if (time_since_switch > 1/(2*f_switch_max))
%             %if(abs(i_q_error)>tol || abs(i_d_error) > tol)
%                 q_dir = v_q_vec;
%                 d_dir = v_d_vec;
%                 theta_v = atan2(q_dir, d_dir);
%                 if(theta_v < 0) theta_v = theta_v + 2*pi; end
%                 theta_s = theta_v + theta;
%                 theta_s = mod(theta_s, 2*pi);
%                 time_since_switch = 0;
%             %end
% 
% 
%         %end
%         
%         stator_vectors = .5*v_bus*[1, .5, -.5, -1, -.5, .5; 0, sqrt(3)/2, sqrt(3)/2, 0, -sqrt(3)/2, -sqrt(3)/2];
%         rot = [cos(-theta), -sin(-theta); sin(-theta), cos(-theta)];
%         error_sf = (rot*[i_d_error; i_q_error]);
%         error_mag = norm(error_sf);
%         K = 200;
%         
%         %sat
%         
%         v_dynamics_sf = rot*[v_d_ff; v_q_ff];
%         voltage_vectors = stator_vectors;% + repmat(v_dynamics_sf,1, 6 );
%         voltage_magnitudes = sqrt(sum(abs(voltage_vectors).^2,1));
%         %error_dot_voltage = error_sf*(voltage_vectors./voltage_magnitudes);
%         error_dot_voltage = error_sf'*voltage_vectors;
%         
%         [val, ind] = max(error_dot_voltage);
%         switch_choice = ind;
%         time_since_switch  = time_since_switch + dt;
%         
%         if (time_since_switch > 1/(2*f_switch_max))
%             if(switch_choice==1) switch_state = [1 0 0];
%                 elseif (switch_choice==2) switch_state = [1 1 0];
%                 elseif (switch_choice==3) switch_state = [0 1 0];
%                 elseif (switch_choice==4) switch_state = [0 1 1];
%                 elseif (switch_choice==5) switch_state = [0 0 1];
%                 elseif (switch_choice==6) switch_state = [1 0 1];
%             end
%             time_since_switch = 0;
%         end
        
        
        
%         if(theta_s < pi/6 || theta_s >= 11*pi/6) switch_state = [1 0 0]; switch_choice = 1;
%             elseif (pi/6 <= theta_s  && theta_s < 3*pi/6 ) switch_state = [1 1 0]; switch_choice = 2;
%             elseif (3*pi/6 <= theta_s && theta_s < 5*pi/6 ) switch_state = [0 1 0]; switch_choice = 3;
%             elseif (5*pi/6 <= theta_s && theta_s < 7*pi/6 ) switch_state = [0 1 1]; switch_choice = 4;
%             elseif (7*pi/6 <= theta_s && theta_s  < 9*pi/6 ) switch_state = [0 0 1]; switch_choice = 5;
%             elseif (9*pi/6 <= theta_s && theta_s  < 11*pi/6 ) switch_state = [1 0 1]; switch_choice = 6;
%         end
% %         
        
        
         
        
        %%% Calculate actual inverter voltages %%%
        v_uvw_cmd = abc_transform*[v_d_cmd; v_q_cmd; 0]; %%2/sqrt(3) deals with svm modulation depth
        
        v_offset = 0.5*(min(v_uvw_cmd) + max(v_uvw_cmd)); %%SVM
        v_uvw_cmd = v_uvw_cmd - v_offset;
        v_uvw_cmd = .5*v_bus + .5*v_uvw_cmd;
        v_uvw = max(min(v_uvw_cmd, v_bus), 0);
        
        
       
        
        count = 0;
    end
    
    if(time_since_switch > switch_delay)
         v_uvw = max(min(v_uvw_cmd, v_bus), 0);
    end
    time_since_switch  = time_since_switch + dt;
         
    v_dq0_actual = dq0_transform*v_uvw;
    count = count+1;
    
    %v_uvw = [0; 0; 0];
    phase_shift = atan2(i_dq0(2), i_dq0(1));
    

    
    %%% Rotor Flux linked to each phase, and derivative %%
    wb_abc_rotor = [wb_r(theta, 0,i_dq0(1)); wb_r(theta, 2*pi/3, i_dq0(2)); wb_r(theta, -2*pi/3, i_dq0(2))];
    wb_abc_rotor_dot = (wb_abc_rotor - wb_abc_rotor_old)*(1/dt);
    
    %%% Phase Inductance and derivative %%%
    l = L(theta, i_dq0(1), i_dq0(2));
    
    %di = abc_transform*i_dq0 - abc_transform*(i_dq0-[1; 1; 0]);
    %dl_di = (l-L(theta, i_dq0(1)-1, i_dq0(2)-1))./[di'; di'; di'];
    dl_dt = (l-l_old)*(1/dt);
    
    %v_sat = dl_di*l*i_dot.*i;
    %%% Back-EMF %%%
    v_bemf = (dl_dt*i + wb_abc_rotor_dot);
    
    
    v_pm = wb_abc_rotor_dot;
    v_rel = dl_dt*i;
    
    
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
    p_mech_abc =  (v_bemf).*i;
    p_mech = sum(p_mech_abc);
    
    p_pm = sum(v_pm.*i);
    p_rel = sum(v_rel.*i);
    
    %%% Phase Torques %%%
    torque_abc = p_mech_abc*(1/thetadot);
    
    %%% Alternate way to calculate torque %%%
    %dwb = [dwb_r(theta, 0); dwb_r(theta, 2*pi/3); dwb_r(theta, -2*pi/3)];
    %dL_abc = dL(theta);
    %t_est = sum((dwb.*i) + (dL_abc)*i);
    
    
    %%% Total Torque %%%
    torque = sum(torque_abc);
    torque_pm = p_pm/thetadot;
    torque_rel= p_rel/thetadot;
    
%    torque = t_est;
    
    
    thetadotdot = 1000;%(torque - B*thetadot)/J;
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
    %v_uvw_cmd_vec(j,:) = v_uvw_cmd';
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
    torque_pm_vec(j) = torque_pm;
    torque_rel_vec(j) = torque_rel;
    v_n_vec(j) = v_n;
    wb_est_vec(j,:) = wb_stator_est';
    l_vec(j,:) = [l(1, 1), l(2, 2), l(3, 3)];
    i_dq_ref_vec(j,:) = [i_d_ref_filt, i_q_ref_filt];
    v_ff_vec(j,:) = [v_d_ff, v_q_ff];
    switch_choice_vec(j) = switch_choice;
    theta_vec(j) = theta;
%    t_est_vec(j,:) = t_est';
    %current_mag_vec(j) = sample_mag;
    
end
toc
uvw_diff_vec = [v_uvw_vec(:,1) - v_uvw_vec(:,2), v_uvw_vec(:,2) - v_uvw_vec(:,3), v_uvw_vec(:,3) - v_uvw_vec(:,1)];

% torque_mech_vec = torque_vec*npp;
% thetadot_mech_vec = thetadot_vec/npp;
% figure;plot(t, torque_vec, t, t_est_vec);
%figure;plot(t, thetadot_vec);
figure;plot(t, i_dq_vec, t, i_dq_ref_vec); legend('id', 'iq', 'id ref', 'iq ref');
%figure;plot(t, v_ff_vec); legend('vd ff', 'vq ff');
%figure;plot(t, torque_vec); title('Torque');
%figure;plot(t, cmd_vec); legend('vd', 'vq')
%figure;plot(t, l_vec); legend('la', 'lb', 'lc');
figure;plot(t, i_vec); legend('ia', 'ib', 'ic');

figure;subplot(2, 1, 1);
plot(t, i_dq_vec, t, i_dq_ref_vec); legend('id', 'iq', 'id ref', 'iq ref');
xlabel('Time (s)'); ylabel('Current (A)');
subplot(2, 1, 2); plot(t, i_vec); legend('ia', 'ib', 'ic');
xlabel('Time (s)'); ylabel('Current (A)');
%figure;plot(thetadot_vec, i_vec);
%figure;plot(t, i_dq_vec); title('I D/Q');
%figure;plot(t, torque_vec); title ('Torque');
%figure;plot(t, uvw_diff_vec); title('Line-To-Line Voltages @300 rad/s'); xlabel('Time (s)'); ylabel('Volts'); legend('U-V', 'V-W', 'W-U');

%hold all; plot(t, torque_pm_vec); plot(t, torque_rel_vec);
%figure;plot(t, thetadot_mech_vec); title('Theta dot');
%figure;plot(thetadot_mech_vec, torque_mech_vec); title('Torque vs Speed');
%figure; plot(thetadot_mech_vec, torque_mech_vec./(((i_dq_vec(:,1).^2)+(i_dq_vec(:,2).^2)).^.5));
%figure;plot(t, v_uvw_vec); title('UVW Voltages');
%figure;plot(thetadot_mech_vec, power_mech_vec); title('Power vs Speed');
%figure;plotyy(t, phase_shift_vec, t, current_mag_vec);title('Current Phase/Mag');
%figure;plot(t, torque_abc_vec);

t_avg = mean(torque_vec);