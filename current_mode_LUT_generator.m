%%% Uses a different motor_fun, which doesn't rely on a controller
%%% converging - super fast

format compact
clear all
%%% Generate a Percent Max Torque lookup table %%%

n = 400;  % Phase resolution
n2 = 50; % Current intervals
n3 = 121; % Velocity intervals
tdot_max = 1500;
mag_max = 200;
v_max = 144/2;
tdot = linspace(0.001, tdot_max, n3)';

phase_min = pi/2;
phase_max = pi;
tol = (phase_max - phase_min)/n;
%phase_max = 2*pi;

pmi = linspace(1, 0.0, n2);   %Percent Max current.  Easier to generate table this way, then interpolate torques

%v_vec = linspace(1, 40, n2);
%phase = linspace(phase_min, phase_max, n);

v_vec = zeros(n3, 1);
v_log_vec = zeros(n2, n3);
phi_max_vec = zeros(n2, n3);
mag_max_vec = zeros(n2, n3);
v_max_vec = zeros(n2, n3);
tdot_vec = zeros(n2, n3);
torque_max_vec = zeros(n2, n3);
tdot_vec = zeros(n2, n3);
i_mag_vec =  zeros(n2, n3);
percent = zeros(n2, n3);

tic

i_wb_ref = [0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200];
wb_lut = [0.0676, 0.0676, 0.0658, 0.0638, 0.0566, 0.0533, 0.0485, 0.0457, 0.0441, 0.0417, 0.0405];
wb_fun = @(iq) interp1(i_wb_ref, wb_lut, iq, 'pchip');


% i_ref = [0; 20; 40; 60; 80; 100; 120; 130; 150; 170; 200];
% ld_lut = 1e-3*[.65; .5785;  .5655; .533; .494; .351; .26; .2301; .20; .17; .14];%+.3e-3; ;
% lq_lut = 1e-3*[1.465; 1.465; 1.465; 1.465; 1.1571; 1.0052; .850; .7; .6; .5; .33];%+.3e-3;
% l_d_fun = @(id, iq)  interp1(i_ref, ld_lut, norm([id, iq]), 'pchip');
% l_q_fun = @(id, iq)  interp1(i_ref, lq_lut, norm([id, iq]), 'pchip');

i_ref = [0; 40; 80; 120; 150; 200];
ld_lut = 1e-3*[.65; .5655;.494;.46; .38;.36];
lq_lut = 1e-3*[1.465; 1.3754; 1.0740;0.550; 0.37; .31];
l_d_fun = @(id, iq) interp1(i_ref, ld_lut, abs(id), 'pchip');
l_q_fun = @(id, iq) interp1(i_ref, lq_lut, abs(iq), 'pchip');

r = .1;
ppairs = 3;
            
parfor j = 1:n2
    %tic
    for k = 1:n3
        %tic
        phi = linspace(phase_min, phase_max, n)';
        delta = phi(2) - phi(1);
        torque = zeros(n, 1);
        v = zeros(n, 1);
        
        for x = 1:n
            i_d = mag_max*pmi(j)*cos(phi(x));
            i_q = mag_max*pmi(j)*sin(phi(x));

            l_d = l_d_fun(i_d, i_q);
            l_q = l_q_fun(i_d, i_q);
            wb = wb_fun(i_q);

            v_d = r*i_d - ppairs*tdot(k)*l_q*i_q;
            v_q = r*i_q + ppairs*tdot(k)*l_d*i_d + ppairs*tdot(k)*wb;

            v(x) = norm([v_d, v_q]);
            torque(x) = ppairs*(3/2)*(wb+(l_d - l_q)*i_d)*i_q;
            %[torque(x), v(x)] = motor_fun_dq(mag_max*pmi(j), phi(x), tdot(k));
        end
        is_valid = v<v_max;
        tmax = max(torque.*is_valid);
        if(tmax>0)
            ind_max = find(torque==tmax);
        else
            torque = 0;
            ind_max = 1;
        end
        ind_max = ind_max(1);
        
        torque_max_vec(j, k) = tmax;
        phi_max_vec(j, k) = phi(ind_max);
        v_max_vec(j, k) = v(ind_max);
        mag_max_vec(j, k) = mag_max*pmi(j);%mag(ind_max);
        tdot_vec(j, k) = tdot(k);
    %toc    
    end
    
    %j
    
end
toc
is_valid = v_max_vec < v_max;
%toc

%figure;plot(tdot, torque_max_vec(1,:)); title('Max Torque')
%figure;plot(tdot, phi_max_vec(1,:));  title('Max Phase')
%figure;plot(tdot, mag_max_vec(1,:)); title('Max Current')
figure; surf(tdot_vec, mag_max_vec, torque_max_vec.*is_valid);
figure;surf(tdot_vec, mag_max_vec, phi_max_vec);
figure;surf(tdot_vec, mag_max_vec, tdot_vec.*torque_max_vec);
% figure;plot(tdot, i_mag_max_vec); title('Max Current')%figure;plot(tdot, t_max_vec, tdot, t_min_vec); title('Max/Min Torque')
%figure;plot(tdot, phi_max_vec, tdot, phi_min_vec);  title('Max/Min Phase')
%figure;plot(tdot, i_mag_max_vec, tdot, i_mag_min_vec); title('Max/Min Current')

%figure; scatter(phase_vec, torque_vec);
data.tdot = tdot_vec;
data.v = mag_max_vec;
data.torque = torque_max_vec;
data.phi = phi_max_vec;
save('Data/Altermotter/data.mat', 'data');
