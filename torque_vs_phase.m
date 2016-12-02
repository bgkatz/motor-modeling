format compact

n = 10;
n2 = 20;
n3 = 10;
tdot_max = 2000;
tdot = linspace(0.01, tdot_max, n3);
phase_min = pi/2;
phase_max = pi;
mag_vec = linspace(1, 200, n2);
mag = sqrt(3/2)*200;
phase = linspace(phase_min, phase_max, n);
torque_vec = zeros(length(n), 3);
phi_max_vec = zeros(length(n2), 1);
torque_max_vec = zeros(length(n2), 1);


for k = 1:n3
    parfor x = 1:n
        [t, t_pm, t_rel] = motor_fun(mag, phase(x), tdot(k));
        torque_vec(x,:) = [t, t_pm, t_rel];
    end
    t_max = max(torque_vec(:,1));
    phi_max = phase(find(torque_vec(:,1) == t_max ));
    torque_max_vec(k) = t_max;
    phi_max_vec(k) = phi_max;
    %figure; plot(phase, torque_vec(:,1));
    k
end

% figure; plot(tdot, phi_max_vec);
% xlabel('Theta dot');
% ylabel('Phase');



% figure;plot(phase, torque_vec);
% title(strcat('Torque Components, ', num2str(mag), ' Amps'));
% legend('Total', 'PM', 'Reluctance');
% xlabel('Current Angle (rad)');
% ylabel('Torque (N-m)');


figure;plotyy(tdot, phi_max_vec, tdot, torque_max_vec);
%data = [tdot', phi_max_vec'];