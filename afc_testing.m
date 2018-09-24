clear all;
l = 15e-5;
r = 15e-3;
dt = 25e-6;

h = 5;
a = .001;

ki = 1-exp(-r*dt/(l));
k = r*((pi/10)/(1-exp(-r*dt/l)));

k_afc = 100;
phi = 0;

i_int = 0;
pi_int = 0;
afc_int1 = 0;
afc_int2 = 0;

t_final = 1;
t = 0:dt:t_final;

i_ref = 10;
theta = 0;
thetadot = 10;
thetadotdot = 0;
i = 0;
v = 0;

i_vec = [];
v_vec = [];
theta_vec = [];
thetadot_vec = [];
afc_int_vec = [];

for x = 1:length(t);
    
    i_error = (i_ref - i)/thetadot;
    i_int = i_int + ki*i_error;
    afc_int1 = afc_int1 + dt*k_afc*cos(h*theta + phi)*i_error;
    afc_int2 = afc_int2 + dt*k_afc*sin(h*theta + phi)*i_error;
    afc_ref = cos(h*theta)*afc_int1 + sin(h*theta)*afc_int2;
    
    
    pi_error = i_ref + thetadot*afc_ref - i;
    pi_int = pi_int + pi_error*ki;
    v_control = k*(pi_error + pi_int) + 0*afc_ref;
    
    v_dist = a*thetadot*sin(h*theta);
    %v_control = k*(i_error + i_int);
    %v_control = 0;
    v = v_control + v_dist;
    
    didt = (v - r*i)/l;
    i  = i + dt*didt;
    thetadot = 10 + 1000*sin(10*t(x));%thetadot + dt*thetadotdot;
    theta = theta + dt*thetadot;
    
    i_vec(x) = i;
    v_vec(x) = v_control;
    theta_vec(x) = theta;
    thetadot_vec(x) = thetadot;
    afc_int_vec(x,:) = [afc_int1, afc_int2];
end


figure;plot(t, i_vec); title('Current');
figure;plot(t, v_vec); title('Voltage');
figure;plot(t, thetadot_vec); title('Velocity');
figure;plot(t, afc_int_vec);