format compact

n = 50;
phase_min = 0;
phase_max = 2*pi;
mag = 200;
phase = linspace(phase_min, phase_max, n);
torque_vec = zeros(length(n), 3);

parfor x = 1:n
    [t, t_pm, t_rel] = motor_fun(mag, phase(x));
    torque_vec(x,:) = [t, t_pm, t_rel];
end

figure;plot(phase, torque_vec);