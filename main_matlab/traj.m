p0=0;
pf0=360;
pf1=-90;
pf2=90;

t=[0:0.01:10-0.01];

T=t(end);

S0 = [pf0;0;0];
S1 = [pf1;0;0];
S2 = [pf2;0;0];

M = [T^5    T^4    T^3;...
     5*T^4  4*T^3  3*T^2;...
     20*T^3 12*T^2 6*T];

X0=M\S0;
X1=M\S1;
X2=M\S2;

A0 = X0(1);
B0 = X0(2);
C0 = X0(3);

A1 = X1(1);
B1 = X1(2);
C1 = X1(3);

A2 = X2(1);
B2 = X2(2);
C2 = X2(3);

t0 = ones(1,length(t));
t2 = t.*t;
t3 = t2.*t;
t4 = t3.*t;
t5 = t4.*t;

tempo = [t5;t4;t3;t2;t;t0];

F=p0;
E=0;
D=0;

q0 = [t;[A0 B0 C0 D E F]*tempo]';
q1 = [t;[A1 B1 C1 D E F]*tempo]';
q2 = [t;[A2 B2 C2 D E F]*tempo]';
qd0 = [t;[5*A0 4*B0 3*C0 2*D E]*tempo(2:end,:)]';
qd1 = [t;[5*A1 4*B1 3*C1 2*D E]*tempo(2:end,:)]';
qd2 = [t;[5*A2 4*B2 3*C2 2*D E]*tempo(2:end,:)]';
qdd0 = [t;[20*A0 12*B0 6*C0 2*D]*tempo(3:end,:)]';
qdd1 = [t;[20*A1 12*B1 6*C1 2*D]*tempo(3:end,:)]';
qdd2 = [t;[20*A2 12*B2 6*C2 2*D]*tempo(3:end,:)]';