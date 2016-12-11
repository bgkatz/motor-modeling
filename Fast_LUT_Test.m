%%% Solver which iterates in on the optimal phase, instead of searching all
%%% phases at max resolution

format compact
clear all
%%% Generate a Percent Max Torque lookup table %%%

n = 200;  % Phase resolution
n2 = 1; % Torque intervals
n3 = 4; % Velocity intervals
tdot_max = 2500;
mag_max = sqrt(3/2)*200;
tdot = linspace(0.01, tdot_max, n3)';

phase_min = pi/2;
phase_max = pi;
tol = (phase_max - phase_min)/n;
%phase_max = 2*pi;

pmi = linspace(0, 1, n2);   %Percent Max current.  Easier to generate table this way, then interpolate torques

mag_vec = linspace(1, sqrt(3/2)*200, n2);
%phase = linspace(phase_min, phase_max, n);


mag_log_vec = zeros(n2*n3, 1);
phi_max_vec = zeros(n2*n3, 1);
torque_max_vec = zeros(n2*n3, 1);
tdot_vec = zeros(n2*n3, 1);
i_mag_vec =  zeros(n2*n3, 1);
percent = zeros(n2*n3, 1);
%%% Find max achievable current and pos/neg torques vs speed %%%
parfor k = 1:n3
    tic
    phi = linspace(phase_min, phase_max, 5)';
    delta = phi(2) - phi(1);
    torque = zeros(5, 1);
    for x = 1:5
        torque(x) = motor_fun(mag_max, phi(x), tdot(k));
    end
    tmax = max(torque);
    ind_max = find(torque==tmax);
    ind_max = ind_max(1);
    ind = [ind_max-1; ind_max; ind_max+1];
    while(delta > tol)
        delta = delta/2;
        
        if(ind(3) > length(phi))
            phi = circshift(phi, -1);
            torque = circshift(torque, -1);
            ind = ind-1;
        elseif (ind(1)==0)
            phi = circshift(phi, 1);
            torque = circshift(torque, 1);
            ind = ind+1;
        end
        
        phi = [phi(ind(1)); phi(ind(1))+delta; phi(ind(2)); phi(ind(2))+delta; phi(ind(3))];
        torque = [torque(ind(1)); motor_fun(mag_max, phi(2), tdot(k)); torque(ind(2)); motor_fun(mag_max, phi(4), tdot(k)); torque(ind(3))];
        tmax = max(torque);
        ind_max = find(torque==tmax);
        ind_max = ind_max(1);
        ind = [ind_max-1; ind_max; ind_max+1];
        %torque_vec = [torque_vec;torque];
        %phase_vec = [phase_vec;phase];
        torque_max_vec(k) = max(torque);
        phi_max_vec(k) = phi(ind_max);
    end   

    toc
end

figure;plot(tdot, torque_max_vec); title('Max Torque')
figure;plot(tdot, phi_max_vec);  title('Max Phase')
% figure;plot(tdot, i_mag_max_vec); title('Max Current')%figure;plot(tdot, t_max_vec, tdot, t_min_vec); title('Max/Min Torque')
%figure;plot(tdot, phi_max_vec, tdot, phi_min_vec);  title('Max/Min Phase')
%figure;plot(tdot, i_mag_max_vec, tdot, i_mag_min_vec); title('Max/Min Current')

%figure; scatter(phase_vec, torque_vec);
