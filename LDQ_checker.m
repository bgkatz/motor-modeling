clear all;
format compact;
format long g;

mag = 50;
phase = linspace(0, 2*pi, 100);

t_pm_vec = zeros(length(phase), 1);
t_rel_vec = zeros(length(phase), 1);

for x = 1:length(phase)
    [t_pm_vec(x), t_rel_vec(x)] = torque_split_fun(mag*cos(phase(x)), mag*sin(phase(x)));
end

figure;plot(phase, t_pm_vec, phase, t_rel_vec, phase, t_pm_vec+t_rel_vec);
legend('pm', 'reluctance', 'total')