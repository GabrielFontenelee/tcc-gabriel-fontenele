function [q,qd,qdd] = trajetoria(p0,delta_pos,t)

% Esta funcao é valida para o caso em que
% a velocidade e aceleração são iguais a zero
% para gerar trajetórias polinomiais
% de ordem 5

T=t(end);

S = [delta_pos;0;0];

M = [T^5    T^4    T^3;...
     5*T^4  4*T^3  3*T^2;...
     20*T^3 12*T^2 6*T];

X=M\S;

A = X(1);
B = X(2);
C = X(3);

t0 = ones(1,length(t));
t2 = t.*t;
t3 = t2.*t;
t4 = t3.*t;
t5 = t4.*t;

tempo = [t5;t4;t3;t2;t;t0];

F=p0;
E=0;
D=0;


q = [t;[A B C D E F]*tempo]';
qd = [t;[5*A 4*B 3*C 2*D E]*tempo(2:end,:)]';
qdd = [t;[20*A 12*B 6*C 2*D]*tempo(3:end,:)]';