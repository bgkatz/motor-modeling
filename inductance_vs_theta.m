clear all;
motorConfig = 'Altermotter';
run(strcat('Motor Configs\', motorConfig));

t = linspace(0, 4*pi, 100);

abc = @(theta) (3/2)*[cos(-theta), sin(-theta), 1/sqrt(2);
    cos((2*pi/3)-theta), sin((2*pi/3)-theta), 1/sqrt(2);
    cos((-2*pi/3)-theta), sin((-2*pi/3)-theta), 1/(sqrt(2))];

dq0 = @(theta) inv(abc(theta));

ld = [];
lq = []
for x = 1:length(t)
    inductance = L(t(x), 0, 0);
    ldq = abc(t(x))*inductance*dq0(t(x));
    ld = [ld; ldq(1, 1)];
    lq = [lq; ldq(2, 2)];
end

figure;plot(t, ld, t, lq);