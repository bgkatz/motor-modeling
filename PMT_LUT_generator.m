format compact

%%% Generate a Percent Max Torque lookup table %%%

n = 100;  % Phase intervals
n2 = 5; % Torque intervals
n3 = 50; % Velocity intervals
tdot_max = 2500;
mag_max = sqrt(3/2)*200;
tdot = linspace(0.01, tdot_max, n3);

phase_min = pi/2;
phase_max = pi;
%phase_max = 2*pi;

pmi = linspace(0, 1, n2);   %Percent Max current.  Easier to generate table this way, then interpolate torques

mag_vec = linspace(1, sqrt(3/2)*200, n2);
phase = linspace(phase_min, phase_max, n);
torque_vec = zeros(length(n), 4);

% mag_log_vec = zeros(length(n2)*length(n3), 1);
% phi_max_vec = zeros(length(n2)*length(n3), 1);
% torque_max_vec = zeros(length(n2)*length(n3), 1);
% tdot_vec = zeros(length(n2)*length(n3), 1);
% i_mag_vec =  zeros(length(n2)*length(n3), 1);


%%% Find max achievable current and pos/neg torques vs speed %%%
for k = 1:n3
    tic
    parfor x = 1:n
        [t, t_pm, t_rel, i_mag] = motor_fun(mag_max, phase(x), tdot(k));
        torque_vec(x,:) = [t, t_pm, t_rel, i_mag];
    end
    t_max = max(torque_vec(:,1));
    t_min = min(torque_vec(:,1));
    
    ind_max = find(torque_vec(:,1) == t_max);
    %ind_min = find(torque_vec(:,1) == t_min);
    phi_max = phase(ind_max);
    %phi_min = phase(ind_min)
    i_mag_max = torque_vec(ind_max,4);
    %i_mag_min = torque_vec(ind_min, 4);
    t_max_vec(k) = t_max;
    %t_min_vec(k) = t_min;
    phi_max_vec(k) = phi_max;
    %phi_min_vec(k) = phi_min;
    i_mag_max_vec(k) = i_mag_max;
    %i_mag_min_vec(k) = i_mag_min;
    i_max_vec(k) = i_mag_max;%max(i_mag_max, i_mag_min);   
    toc
    k
end

figure;plot(tdot, t_max_vec); title('Max Torque')
figure;plot(tdot, phi_max_vec);  title('Max Phase')
figure;plot(tdot, i_mag_max_vec); title('Max Current')%figure;plot(tdot, t_max_vec, tdot, t_min_vec); title('Max/Min Torque')
%figure;plot(tdot, phi_max_vec, tdot, phi_min_vec);  title('Max/Min Phase')
%figure;plot(tdot, i_mag_max_vec, tdot, i_mag_min_vec); title('Max/Min Current')
%%
for k = 1:n3
    for j = 1:n2
        parfor x = 1:n
            [t, t_pm, t_rel, i_mag] = motor_fun(i_max_vec(k)*pmi(j), phase(x), tdot(k));
            torque_vec(x,:) = [t, t_pm, t_rel, i_mag];
        end
        t_max = max(torque_vec(:,1));
        %t_min = min(torque_vec(:,1));
        ind_max = find(torque_vec(:,1) == t_max);
        ind_max = ind_max(1);
        %ind_min = find(torque_vec(:,1) == t_min);
        %ind_min = ind_min(1);
        phi_max = phase(ind_max);
        %phi_min = phase(ind_min);
        t_max_mat(k, j) = t_max;
        %t_min_mat(k, j) = t_min;
        phi_max_mat(k, j) = phi_max;
        %phi_min_mat(k, j) = phi_min;
    end
    k
end

[pmi_grid, tdot_grid] = meshgrid(pmi, tdot);
figure; surf(tdot_grid, pmi_grid, t_max_mat); title('Max torque');
%figure; surf(tdot_grid, pmi_grid, t_min_mat); title('Min torque');
figure; surf(tdot_grid, pmi_grid, phi_max_mat); title('Max phase');
%figure; surf(tdot_grid, pmi_grid, phi_min_mat); title('Min phase');
%figure; surf(tdot_grid, pmi_grid, t_max_mat); surf(tdot_grid, pmi_grid, t_min_mat); hold all; title('Torque Envelope');

data.tmax = t_max_mat;
%data.t_min = t_min_mat;
data.phi_max = phi_max_mat;
%data.phi_min = phi_min_mat;
data.pmi = pmi_grid;
data.tdot = tdot_grid;

% figure;plotyy(tdot, phi_max_vec, tdot, torque_max_vec);
% data = [tdot', phi_max_vec'];
% save('2d_lut.mat', 'data');

% figure;scatter3(tdot_vec, mag_log_vec, phi_max_vec, 20, phi_max_vec, 'filled'); 
% view(0, 90); colorbar();
% xlabel('Rad/s'); ylabel('Current Command'); zlabel('Phase');
% figure;scatter3(tdot_vec, mag_log_vec, torque_max_vec, 20, torque_max_vec, 'filled'); 
% view(0, 90); colorbar();
% xlabel('Rad/s'); ylabel('Current Command'); zlabel('Torque');
% figure;scatter3(tdot_vec, i_mag_vec, torque_max_vec, 20, torque_max_vec, 'filled'); 
% view(0, 90); colorbar();
% xlabel('Rad/s'); ylabel('Current'); zlabel('Torque');
% 
% data = [tdot_vec, mag_log_vec, i_mag_vec, phi_max_vec, torque_max_vec];
% save('2d_lut.mat', 'data');