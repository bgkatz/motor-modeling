clear all
syms theta dtheta ia ib ic dia dib dic va vb vc La Lb Lc Lm Wpm Wba Wbb Wbc Ld Lq L0 id iq i0 r Ls
assume([theta dtheta ia ib ic dia dib dic va vb vc La Lb Lc Lm Wpm Wba Wbb Wbc Ld Lq L0 id iq i0 r Ls], 'real');

i = [ia; ib; ic];
idq = [id; iq; i0];
di = [dia; dib; dic];

Ldq = [Ld, 0, 0;
        0, Lq, 0;
        0, 0, L0];
    
Wba = Wpm*cos(-theta);
Wbb = Wpm*cos(2*pi/3 - theta);
Wbc = Wpm*cos(-2*pi/3-theta);
Wb = [Wba; Wbb; Wbc];

La = .5*(Ld+Lq) + .5*(Ld-Lq)*cos(-2*theta);
Lb = .5*(Ld+Lq) + .5*(Ld-Lq)*cos(2*(2*pi/3-theta));
Lc = .5*(Ld+Lq) + .5*(Ld-Lq)*cos(2*(-2*pi/3-theta));

L = [Ls, 0, 0;
    0, Ls, 0;
    0, 0, Ls];

R = [r 0 0;
    0 r 0;
    0 0 r;];

dL = diff(L, theta);
dW = diff(Wb, theta);

abc = [cos(-theta), sin(-theta), 1/sqrt(2);
    cos((2*pi/3)-theta), sin((2*pi/3)-theta), 1/sqrt(2);
    cos((-2*pi/3)-theta), sin((-2*pi/3)-theta), 1/(sqrt(2))];
%dq0 = inv(abc);
dq0 = (2/3)*[cos(theta), cos(theta - 2*pi/3), cos(theta+2*pi/3);
            -sin(theta), -sin(theta-2*pi/3), -sin(theta+2*pi/3);
            .5, .5, .5];

v = L*di + dtheta*dW + R*i;
vdq = dq0*v;
simplvdq = simplify(vdq);

vd = vdq(1)
vq = vdq(2)
v0 = vdq(3);

