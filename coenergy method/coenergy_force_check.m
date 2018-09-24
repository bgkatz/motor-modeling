clear all;
i_ref = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
l_lut = [.5, .5, .5, .5, .4, .3, .2, .1, .1, .1];
L = @(theta, i) (1-interp1(i_ref, l_lut, i, 'pchip').*cos(2*theta))*.001;
wb_lut = [.01, .01, .01, .01, .009, .008, .007, .006, .006, .006];
Wb = @(theta, i) interp1(i_ref, wb_lut, i, 'pchip').*cos(theta);

i = linspace(1, 10, 10);
theta = linspace(0, 2*pi, 1000);
f = [];

tic
for j = 1:length(theta)
    for k = 1:length(i)
        f(j, k) = calc_force(theta(j), i(k), L, Wb);
    end
end
toc
figure;plot(theta, f);
xlabel('Electrical Angle (Rad)');
ylabel('Force (N)');
NicePlot;