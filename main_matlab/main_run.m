clear all
close all
clc

% =========================== TABELA DE SIMULAÇÕES
% simulacao = "TCC" -> (DINÂMICA DIRETA) Possibilidade de aplicar os 3 métodos de controle

% Versão matlab
% matlab_version = 2024 -> Usuários de Matlab 2024b
% matlab_version = 2023 -> Usuários de Matlab 2023b

simulacao = "TCC";
matlab_version = 2024;
Fullfilename = strcat("sim_",string(simulacao),'_',string(matlab_version),'.slx');

% ============================  FLAGS PARA ATIVAÇÃO
ILC_ON = 1;
NOISE_ON = 0; % Ativa ruído nas medições de posição e de velocidade de cada link. Amplitude de 1e-4
DISTURBANCE_ON = 1;
FREEZE_ON = 0;
IC_ERROR_ON = 0;
SATURATION_ON = 1;
PAYLOAD_ON = 0;
m_payload = 0.125;

fprintf('ILC_ON = %d \n',ILC_ON);
fprintf('FREEZE_ON = %d \n',FREEZE_ON);
fprintf('NOISE_ON = %d \n',NOISE_ON);
fprintf('DISTURBANCE_ON = %d \n',DISTURBANCE_ON);
fprintf('IC_ERROR_ON = %d \n',IC_ERROR_ON);
fprintf('SATURATION_ON = %d \n',SATURATION_ON);
fprintf('Filename = %s \n',Fullfilename);


% ============================  SETUP RUÍDOS

noise_power = [1e-10,1e-10,1e-10,1e-10,1e-10,1e-10]; % Potência do sinal de ruído branco em cada link  [q0,qd0,q1,qd1,q2,qd2]
t0_noise = 800;
tf_noise = 900;

% ============================  SETUP SATURAÇÃO

Lim_Torque_motor = 1; % [N*m]


% ============================  SETUP PERTUBAÇÕES

dist = [0.5*Lim_Torque_motor,0.5*Lim_Torque_motor,0.5*Lim_Torque_motor]; % Amplitude das pertubações em cada motor [u0,u1,u2]
t0_dist = 26; % Tempo inicial para aplicação das pertubações [em segundos] 
tf_dist = 32; % Tempo final para aplicação das pertubações [em segundos]

% ============================  SETUP FREEZE

t0_freeze = 10;
tf_freeze = 11;
    
% ============================   SETUP ERROS DE CONDIÇÃO INICIAL 

a = 10; 
b = 15; 
r = a + (b - a) * rand(1,3);

error_difference = [r(1),0,r(2),0,r(3),0];   % [q0[m],qd0[m/s],q1[m],qd1[m/s],q2[m],qd2[m/s]]

% ============================  PARAMETROS DE TEMPO DE SIMULAÇÃO 

Tsim = 50;
Ts = 0.01;  
Ts_sim = Ts/10;

% ============================ SETUP DO GERADOR DE TRAJETORIA

Tspan = 10;
end_learning_time = 5 * Tspan;
shift_time = Tspan/2;

t_pick = 0:Ts:Tspan/2-Ts; % Time to pick
t_return = 0:Ts:Tspan/2-Ts; % Time to return
t = 0:Ts:Tspan;

[q0_ida,qd0_ida,qdd0_ida] = trajetoria(0,120,t_pick); 
[q1_ida,qd1_ida,qdd1_ida] = trajetoria(0,-90,t_pick);
[q2_ida,qd2_ida,qdd2_ida] = trajetoria(0,90,t_pick);

[q0_volta,qd0_volta,qdd0_volta] = trajetoria(120,-120,t_return); 
[q1_volta,qd1_volta,qdd1_volta] = trajetoria(-90,90,t_return);
[q2_volta,qd2_volta,qdd2_volta] = trajetoria(90,-90,t_return);

% ============================ 

%Ganho dos integradores no modelo do FK
G1 = 1;
G2 = 0.9;

%Parâmetros do controlador

gama1=100;
gama2=1;
gama3=1;

Kp0 = 30;
Kp1 = 30;
Kp2 = 30;

Kv0 = 25/10;
Kv1 = 25/10;
Kv2 = 25/10;


sim(Fullfilename)


t = juntas.Time;
P0 = juntas.Data(:,1);
p0 = juntas.Data(:,2);
P1 = juntas.Data(:,3);
p1 = juntas.Data(:,4);
P2 = juntas.Data(:,5);
p2 = juntas.Data(:,6);

t0 = u0.Time;
u0 = u0.Data;
t1 = u1.Time;
u1 = u1.Data;
t2 = u2.Time;
u2 = u2.Data;

tg = t;
Kq0 = ganhos.Data(:,1)*ones(1,length(tg));
Kq1 = ganhos.Data(:,2)*ones(1,length(tg));
Kq2 = ganhos.Data(:,3)*ones(1,length(tg));
Kw0 = ganhos.Data(:,4)*ones(1,length(tg));
Kw1 = ganhos.Data(:,5)*ones(1,length(tg));
Kw2 = ganhos.Data(:,6)*ones(1,length(tg));

plot(t,P0,'-.b',...
     t,P1,'-.k',...
     t,P2,'-.r','LineWidth',3);
hold on
plot(t,p0,'-b',...
     t,p1,'-k',...
     t,p2,'-r','LineWidth',2);
grid on
title('Joint Position')
xlabel('Time (s)')
legend('\rho*',...
    '\theta_1*',...
    '\theta_2*',...
    '\rho',...
    '\theta_1',...
    '\theta_2')
fontname(gcf,"Times New Roman")
axes('position',[.2 .6 .3 .3])

box on                                                        % put box around new pair of axes
indexOfInterest = (t < 5.25) & (t > 4.25);                    % range of t near perturbation
plot(t(indexOfInterest),p0(indexOfInterest),'LineWidth',2)    % plot on new axes
axis tight
grid on

figure
plot(t0,u0,'-b',...
     t1,u1,'-k',...
     t2,u2,'-r','LineWidth',1);
grid on
title('Joint torques')
xlabel('Time (s)')
legend('\tau(\rho)',...
    '\tau(\theta_1)',...
    '\tau(\theta_2)')
fontname(gcf,"Times New Roman")

F0=tf([Kp0 Kv0],[1 1000]);
F1=tf([Kp1 Kv1],[1 1000]);
F2=tf([Kp2 Kv2],[1 1000]);

figure
w=[0.01:0.01:10];
bode(w,F0)
hold on
bode(w,F1)
bode(w,F2)
grid on
clc

figure
subplot(3,1,1)
plot(t,p0-P0,'LineWidth',2,'Color',[0 0.4470 0.7410])
ylim([-5 5])
grid on
title('error \rho')
subplot(3,1,2)
plot(t,p1-P1,'LineWidth',2,'Color',[0.9 0.7 0])
ylim([-3 3])
grid on
title('error \theta_1')
subplot(3,1,3)
plot(t,p2-P2,'LineWidth',2,'Color',[0.4470 0.7410 0])
ylim([-3 3])
xlabel('Time (s)')
grid on
title('error \theta_2')
fontname(gcf,"Times New Roman")

Kq0(1)=30;
Kq1(1)=30;
Kq2(1)=30;
Kw0(1)=2.5;
Kw1(1)=2.5;
Kw2(1)=2.5;

figure
subplot(3,2,1)
plot(tg,Kq0,'LineWidth',2,'Color',[0 0 0.9])
title('K_p0')
grid on
xlabel('Time (s)')
subplot(3,2,2)
plot(tg,Kw0,'LineWidth',2,'Color',[0 0.5 0.0])
title('K_v0')
grid on
xlabel('Time (s)')
subplot(3,2,3)
plot(tg,Kq1,'LineWidth',2,'Color',[0 0 0.9])
title('K_p1')
grid on
xlabel('Time (s)')
subplot(3,2,4)
plot(tg,Kw1,'LineWidth',2,'Color',[0 0.5 0])
title('K_v1')
grid on
xlabel('Time (s)')
subplot(3,2,5)
plot(tg,Kq2,'LineWidth',2,'Color',[0 0 0.9])
title('K_p2')
grid on
xlabel('Time (s)')
subplot(3,2,6)
plot(tg,Kw2,'LineWidth',2,'Color',[0 0.5 0])
title('K_v2')
grid on
xlabel('Time (s)')
fontname(gcf,"Times New Roman")
