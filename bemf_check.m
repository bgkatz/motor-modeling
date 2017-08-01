

%%% Load Motor Configuration %%%
motorConfig = 'CheetahMotor';
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

i_dq0 = [0; 0];

r_s = r_a;
loop_dt = 1/f_switch;


%%% Initialize Dynamics Variables %%%
i = [0; 0; 0];
v = [0; 0; 0];
v_bemf = v;
theta = 0;
thetadot = 603;
thetadotdot = 0;
phase_shift = 0;

tfinal = 1;
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
    elseif (strcmp(termination, 'wye'))
        value = [v_bemf;0];
        v_n = value(4);
        v_uvw = v_bemf + [v_n; v_n; v_n];
    elseif (strcmp(termination, 'ind'))
        v = v_uvw;
    end
    
    
    
    thetadot = thetadot + thetadotdot;%*dt;
    thetadot_mech = thetadot;
    
    theta = theta + thetadot*dt;

   
    wb_abc_rotor_old = wb_abc_rotor;
    l_old = l;
    
    %%% Save Data %%%
    thetadot_vec(j) = thetadot;
    i_vec(j,:) = i';
    v_vec(j,:) = v'; 
    v_uvw_vec(j,:) = v_uvw';
    v_bemf_vec(j,:) = v_bemf';
    thetadot_mech_vec(j) = thetadot_mech;
    v_n_vec(j) = v_n;
    %current_mag_vec(j) = sample_mag;
end
toc

%figure;plot(t, i_vec);
figure;plot(t, v_uvw_vec(:,1) - v_uvw_vec(:,2));
