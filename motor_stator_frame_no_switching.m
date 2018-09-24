%%% This version has no switching dynamics, so can be much faster.  Usefull
%%% for long time-scale modeling, or if switching speed effects don't
%%% matter

%%% Load Motor Configuration %%%
motorConfig = 'U12';
run(strcat('Motor Configs\', motorConfig));

%%% Transforms %%%
%%% Power-invariant form %%%
%%% Not your canonical transform, but it fits my assumptions %%%
abc = @(theta) [cos(-theta), sin(-theta), 1/sqrt(2);
    cos((2*pi/3)-theta), sin((2*pi/3)-theta), 1/sqrt(2);
    cos((-2*pi/3)-theta), sin((-2*pi/3)-theta), 1/(sqrt(2))];

dq0 = @(theta) inv(abc(theta)); %= inv(abc)

%%% Inverter Properties %%%
spl = 10;            %%simulation steps per loop
f_switch = 40000;       %% maximum switching frequency
f_loop = 40000;    %%Loop frequency
f_sim = spl*f_loop;
v_bus = 15;         %%Bus voltage
loop_dt = 1/f_loop;
switch_delay = .0*loop_dt;

%%% Current Controller %%%

%i_ref = 200*sqrt(2);
%phase_ref = 2;
%i_q_ref = i_ref*sin(phase_ref);
%i_d_ref = i_ref*cos(phase_ref);

%i_q_ref = 56.59;
%i_d_ref = -101.5
pmt_old = 0;

i_dq0 = [0; 0;0];
i_dq0_old = [0;0; 0];
i_dq0_dot = [0;0; 0];

r_s = r_a;



ki_q = 1-exp(-r_s*loop_dt/(l_q));
k_q = r_s*((pi/6)/(1-exp(-r_s*loop_dt/l_q)));
ki_d = 1-exp(-r_s*loop_dt/(l_d));
k_d = r_s*((pi/6)/(1-exp(-r_s*loop_dt/l_d)));


% k_q = 2.5;
% ki_q = .01;
% k_d = 2;
% ki_d = .025;

i_q_ref = 0;
i_d_ref = 0;
        
q_int = 0;
d_int = 0;
q_int_max = v_bus*2/sqrt(3);
d_int_max = v_bus*2/sqrt(3);
mag_int = 0;
mag_int_max = v_bus;
i_q_ref_filt = 0;
i_d_ref_filt = 0;
theta_s = 0;

param_est = [r_s+.5; 2*l_d; .5*l_q; .4*k1];
%param_est = [r_s+.5; .4*k1];

A = rand(4);
b = [1; 1; 1; 1];

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
J = .00001; %%Kg-m^2
B  = 0.000001; %%N-m*s/rad

%%% Initialize Conditions %%%
i = [0; 0; 0];
v = [0; 0; 0];
v_bemf = v;
theta = 0.01;
thetadot = 4000;
thetadotdot = 0;%.025;
phase_shift = 0;

tfinal = .025;
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
i_mag_vec = zeros(length(t), 1);
v_uvw_vec = zeros(length(t), 3);
v_uvw_cmd_vec = zeros(length(t), 3);
wb_est_vec = zeros(length(t), 3);
t_est_vec = zeros(length(t), 3);
l_vec = zeros(length(t), 3);
v_ff_vec = zeros(length(t), 2);
switch_choice_vec = zeros(length(t), 1);
theta_vec = zeros(length(t), 1);
param_est_vec = zeros(length(t), 4);
harmonic_vec = zeros(length(t), 2);
dq0_bemf_vec = zeros(length(t), 3);
afc_int_vec = zeros(length(t), 4);
tic

%sv = linspace(0, pmt_lut.speed(end), length(pmt_lut.pmt(1,:)));
%pv = linspace(0, 1, length(pmt_lut.pmt(:,1)));
        
loop_start_time = 0;


n_linreg = 1000;
n_steps = 10;
b_vec = zeros(2*n_linreg, 1);
A_mat = zeros(2*n_linreg, 4);
linreg_counter = 0;
v_d_cmd_old = 0;
v_q_cmd_old = 0;
delay = tfinal*.5;
v_d_cmd = .5;
v_q_cmd = .5;


afc_d_int_1 = 0;
afc_d_int_2 = 0;
afc_q_int_1 = 0;
afc_q_int_2 = 0;
afc_q_int_3 = 0;
afc_q_int_4 = 0;
afc_q_int_5 = 0;
afc_q_int_6 = 0;

for j=1:length(t)
    time = t(j);
    
    %if(time > tfinal/6);
        %tau_ext = 1000;
        %i_q_ref = 20;
        i_d_ref = -10;
        i_q_ref = 2;
    %end
   %%if(time>tfinal/2);
   %   i_d_ref = -5;
   %   i_q_ref = -2;
   %end
   
   %i_q_ref = 3*sin(200*time);
   %i_d_ref = 3*cos(200*time);
   %i_d_ref = i_d_ref + 1e-4;
   %i_q_ref = i_q_ref + 1e-4;
   %i_q_ref = 2;
   %i_d_ref = 2;
    
    i_q_max = ((1/sqrt(3))*v_bus - thetadot*k1)/(r_s);
    i_q_min = (-(1/sqrt(3))*v_bus - thetadot*k1)/(r_s);
    %i_q_ref = max(min(i_q_max, i_q_ref), i_q_min);
        
    
    %%% Sample Current %%%
    if (strcmp(termination, 'delta'))
            i_sample = [i(1)-i(3); i(2) - i(1); i(3) - i(2)];
        elseif (strcmp(termination, 'wye'))
            i_sample = i + .06*(rand(3, 1)-.5);
        elseif (strcmp(termination, 'ind'))
            i_sample = i;
        end

        %%% Calculate Transform Matrix %%%
        dq0_transform = dq0(theta);
        abc_transform = abc(theta+.5*thetadot*loop_dt);

        i_dq0 = dq0_transform*i_sample;
        
        dq0_2 = dq0(5*theta);
        i_harmonic = dq0_2*i_dq0;
        
        
    %control loop
    if(count == (spl))
        
        
        
        
        loop_start_time = time;
        %v_q_coupling = 2*l_d*i_dq0(1)*thetadot;
        %v_d_coupling = -2*l_q*i_dq0(2)*thetadot;
        dq0_bemf = dq0_transform*v_bemf; 
        %%% Controller %%%
        %k_q = 0;
        %k_d = 0;
        %%% Normal PI controller w/ feedforward decopuling and bemf feedforward %%%
        k_db = .5*l_d*f_loop;
        
        
        
        i_q_ref_new = .2*i_q_ref + .8*i_q_ref_filt;
        i_d_ref_new = .2*i_d_ref + .8*i_d_ref_filt;
        
        i_q_dot_ref = (i_q_ref_new - i_q_ref_filt)/loop_dt;
        i_d_dot_ref = (i_d_ref_new - i_d_ref_filt)/loop_dt;
        
        i_q_ref_filt = i_q_ref_new;
        i_d_ref_filt = i_d_ref_new;
        
        i_q_ac = i_q_ref*.7*sin(12*theta);
        
        i_q_error = (i_q_ref + i_q_ac - i_dq0(2))/thetadot;
        i_d_error = (i_d_ref - i_dq0(1))/thetadot;
        
        %q_int = q_int + i_q_error*ki_q*k_q;
        %d_int = d_int + i_d_error*ki_d*k_d;
        
        
        
        k_afc = 00;
        h = 6;
        phi = 0;
        afc_d_int_1  = afc_d_int_1 + loop_dt*k_afc*i_d_error*cos(h*theta + phi);
        afc_d_int_2  = afc_d_int_2 + loop_dt*k_afc*i_d_error*sin(h*theta + phi);
        afc_d_err = afc_d_int_1*cos(h*theta) + afc_d_int_2*sin(h*theta);
        afc_q_int_1  = afc_q_int_1 + loop_dt*k_afc*i_q_error*cos(h*theta + phi);
        afc_q_int_2  = afc_q_int_2 + loop_dt*k_afc*i_q_error*sin(h*theta + phi);
        afc_q_err = afc_q_int_1*cos(h*theta) + afc_q_int_2*sin(h*theta);
        
        h2 = 12;
        afc_q_int_3  = afc_q_int_3 + loop_dt*k_afc*i_q_error*cos(h2*theta + phi);
        afc_q_int_4  = afc_q_int_4 + loop_dt*k_afc*i_q_error*sin(h2*theta + phi);
        afc_q_err_2 = afc_q_int_3*cos(h2*theta) + afc_q_int_4*sin(h2*theta);

        
        
        i_q_error = i_q_ref  + thetadot*afc_q_err + thetadot*afc_q_err_2  - i_dq0(2);
        i_d_error = i_d_ref + thetadot*afc_d_err - i_dq0(1);
        
        q_int = q_int + i_q_error*ki_q;
        d_int = d_int + i_d_error*ki_d;

        %q_int = q_int + afc_q_err*ki_q*k_q;
        %d_int = d_int + afc_d_err*ki_d*k_d;

        q_int = max(min(q_int, q_int_max), -q_int_max);
        d_int = max(min(d_int, d_int_max), -d_int_max);
         
        int_mag = norm([q_int, d_int]);
        
        i_dq0_dot = (i_dq0 - i_dq0_old)/(1.0*loop_dt);
        i_dq0_old = i_dq0;
        
       A = [A(3, 1), A(3, 2), A(3, 3), A(3, 4);
            A(4, 1), A(4, 2), A(4, 3), A(4, 4);
            i_dq0(1), i_dq0_dot(1), -thetadot*i_dq0(2), 0;
            i_dq0(2), thetadot*i_dq0(1), i_dq0_dot(2), thetadot];
        %A = A+1e-6*rand(4);
        b = [b(3); b(4);v_d_cmd; v_q_cmd];
        
        %b = [v_d_cmd; v_q_cmd];
        
        
       % x = A\b;
        

        %param_est = .99*param_est + .01*x*.5;
        
        
        
        if(time>delay)
        if((linreg_counter>0)&(linreg_counter<=n_linreg))
            b_vec(2*(linreg_counter-1)+1) = v_d_cmd;
            b_vec(2*(linreg_counter-1)+2) = v_q_cmd;
            A_mat(2*(linreg_counter-1)+1, :) = [i_dq0(1), 0, -thetadot*i_dq0(2), 0];
            A_mat(2*(linreg_counter-1)+2, :) = [i_dq0(2), thetadot*i_dq0(1), 0, thetadot];
        end
        linreg_counter  = linreg_counter + 1;
        end
        

            
        v_d_ff = 2*i_d_ref_filt*r_a - 2*thetadot*l_q*i_dq0(2) + 0*l_d*i_d_dot_ref;% + k_db*i_d_error;
        v_q_ff = 2*i_q_ref_filt*r_a + 2*k1*thetadot + 2*thetadot*l_d*i_dq0(1) + 0*l_q*i_q_dot_ref;% + k_db*i_q_error;
        %v_d_ff = 0;
        %v_q_ff = 0;
        
        v_q_cmd = k_q*(i_q_error) + q_int + v_q_ff;% +sqrt(3)*r_a*i_q_ref + sqrt(3/2)*thetadot*l_d_nom*i_dq0(1) + sqrt(3)*thetadot*wb_nom;%k_q*i_q_error + q_int;% + v_q_ff;% + v_q_coupling;% + dq0_bemf(2);
        v_d_cmd = k_d*(i_d_error) + d_int + v_d_ff;% + sqrt(3)*r_a*i_d_ref - sqrt(3/2)*thetadot*l_q_nom*i_dq0(2);%k_d*i_d_error + d_int;% + v_q_ff;% + v_d_coupling;% + dq0_bemf(1);
        
        
        
        %v_q_cmd = v_q_ff;
        %v_d_cmd = v_d_ff;
        %%% Limit voltage commands to not overmodulate %%%

        cmd_mag = norm([v_d_cmd, v_q_cmd]);
        %%% Limit voltage commands to not overmodulate too hard %%%
        if(cmd_mag > ((2/sqrt(3))*v_bus))
           v_d_cmd = v_d_cmd*((2/sqrt(3))*v_bus/cmd_mag);
           v_q_cmd = v_q_cmd*((2/sqrt(3))*v_bus/cmd_mag);
        end
        
         
        
        %%% Calculate actual inverter voltages %%%
        v_uvw_cmd = abc_transform*1.27*[v_d_cmd; v_q_cmd; 0]; %%2/sqrt(3) deals with svm modulation depth
        
        v_offset = 0.5*(min(v_uvw_cmd) + max(v_uvw_cmd)); %%SVM
        v_uvw_cmd = v_uvw_cmd - v_offset;
        v_uvw_cmd = .5*v_bus + .5*v_uvw_cmd;
        %v_uvw = max(min(v_uvw_cmd, v_bus), 0);
        
        
       
        
        count = 0;
    end
    
    time_since_switch = time-loop_start_time;
    %if(time_since_switch > switch_delay)
         v_uvw = max(min(v_uvw_cmd, v_bus), 0);
    %end
         
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
    torque = npp*sum(torque_abc);
    torque_pm = p_pm/thetadot;
    torque_rel= p_rel/thetadot;
    
%    torque = t_est;
    
    
    thetadotdot = 0;%(torque - B*thetadot)/J;
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
    i_dq_ref_vec(j,:) = [i_d_ref, i_q_ref];
    v_ff_vec(j,:) = [v_d_ff, v_q_ff];
    %switch_choice_vec(j) = switch_choice;
    theta_vec(j) = theta;
%    t_est_vec(j,:) = t_est';
    i_mag_vec(j) = (i_dq0(1)^2 + i_dq0(2)^2)^.5;
    param_est_vec(j,:) = param_est';
    i_harmonic_vec(j,:) = [i_harmonic(1), i_harmonic(2)];
    dq0_bemf_vec(j,:) = dq0_bemf';
    afc_int_vec(j,:) = [afc_d_int_1, afc_d_int_2, afc_q_int_1, afc_q_int_2];
    
end
toc
uvw_diff_vec = [v_uvw_vec(:,1) - v_uvw_vec(:,2), v_uvw_vec(:,2) - v_uvw_vec(:,3), v_uvw_vec(:,3) - v_uvw_vec(:,1)];

% torque_mech_vec = torque_vec*npp;
% thetadot_mech_vec = thetadot_vec/npp;
% figure;plot(t, torque_vec, t, t_est_vec);
%figure;plot(t, thetadot_vec);
figure;plot(t, i_dq_vec); legend('id', 'iq'); hold all; plot(t, mod(theta_vec, 2*pi))
%figure;plot(t, i_dq_vec, t, i_dq_ref_vec);legend('id', 'iq', 'id ref', 'iq ref');
%figure;plot(t, v_ff_vec); legend('vd ff', 'vq ff');
%figure;plot(t, torque_vec); title('Torque');
%figure;plot(t, cmd_vec); legend('vd', 'vq')
%figure;plot(t, l_vec); legend('la', 'lb', 'lc');
figure;plot(t, i_vec); legend('ia', 'ib', 'ic');
figure;plot(t, v_uvw_vec); legend('v_u', 'v_v', 'v_w');
%figure;plot(t, afc_int_vec);
%figure;plot(t, i_harmonic_vec);
%figure;plot(t, uvw_diff_vec);

% figure;subplot(2, 1, 1);
% plot(t, i_dq_vec, t, i_dq_ref_vec); legend('id', 'iq', 'id ref', 'iq ref');
% xlabel('Time (s)'); ylabel('Current (A)');
% subplot(2, 1, 2); plot(t, i_vec); legend('ia', 'ib', 'ic');
% xlabel('Time (s)'); ylabel('Current (A)');
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

% figure;
% ax1 = subplot(4, 1, 1);
% title('Phase resistance'); ylabel('Ohms');
% hold all
% plot(t(3000:end), param_est_vec((3000:end),1));
% plot(t(3000:end), r_a*ones(size(t(3000:end))));
% ax2 = subplot(4, 1, 2);
% title('D-axis inductance'); ylabel('Henries');
% hold all
% plot(t(3000:end), param_est_vec((3000:end),2));
% plot(t(3000:end), l_d*ones(size(t(3000:end))));
% ax3 = subplot(4, 1, 3);
% title('Q-axis inductance'); ylabel('Henries');
% hold all
% plot(t(3000:end), param_est_vec((3000:end),3));
% plot(t(3000:end), l_q*ones(size(t(3000:end))));
% ax4 = subplot(4, 1, 4);
% title('Flux Linkage'); ylabel('Webers');
% hold all
% plot(t(3000:end), param_est_vec((3000:end),4));
% plot(t(3000:end), k1*ones(size(t(3000:end))));
% linkaxes([ax1, ax2, ax3, ax4], 'x');
% NicePlot

t_avg = mean(torque_vec);

p_est = A_mat\(.5*b_vec)
p_actual = [r_a; l_d; l_q; k1];
error =  (p_actual - p_est)./p_actual