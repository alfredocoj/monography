%% UNIVERSIDADE ESTADUAL DO MARANHÃO
%% CURSO DE ENGENHARIA DA COMPUTAÇÃO
%% TRABALHO DE CONCLUSÃO DE CURSO
%% TÍTULO: PROJETO DE UM FILTRO DE KALMAN EM UM MANIPULADOR ROBÓTICO USANDO UM ALGORITMO GENÉTICO

%%              CÓDIGO 1: PROJETO DO FILTRO DE KALMAN

clear all
clc
close all

%% 1) Create the state space system by typing the following in the MATLAB Command Window:
A = [0 1 0 0; -2 -2 1 1; 0 0 0 1; 1 1 -1 -1]
B = [0 1 0 0]'
C = [0 0 1 0]
D = [0]


%% Algorítmo Genético - Nº de Gerações: 500

% Q = [0.7353    2.5195    2.4865    2.4677;
%     2.5195   44.7873    5.0790    5.1887;
%     2.4865    5.0790   28.9306    4.9690;
%     2.4677    5.1887    4.9690   26.7117]
% %Q = real(jordan(Q))
% 
% R = [17.5211]

Q = [ 16.7449    2.0101    2.5171    2.6623;
    2.0101   45.0012    5.3850    5.5377;
    2.5171    5.3850   37.6232    4.0802;
    2.6623    5.5377    4.0802   23.8086]

R =    14.4476;

% Q = [ 0.9745    2.4628    2.5202    2.7828;
%     2.4628   45.0011    5.3850    5.5377;
%     2.5202    5.3850   37.6231    4.0802;
%     2.7828    5.5377    4.0802   23.8086]
% 
% R =   14.4476;

[K,P,e] = lqr(A,B,Q,R)

% Modelo no espaço de estados para o Projeto LQR considerando condições
% iniciais
sys = ss(A-B*K, eye(4), eye(4), eye(4));

%% Plot do Projeto LQR
t = 0:0.01:20;
x = initial(sys,[1;0;0;0],t);
x1 = [1 0 0 0]*x';
x2 = [0 1 0 0]*x';
x3 = [0 0 1 0]*x';
x4 = [0 0 0 1]*x';

figure(1);
title('Resposta projeto LQR para condicoes iniciais')
subplot(1,4,1); plot(t,x1), grid
title('Resposta projeto LQR para condicoes iniciais')
ylabel('state variable x1')
subplot(1,4,2); plot(t,x2),grid
title('Resposta projeto LQR para condicoes iniciais')
ylabel('state variable x2')
subplot(1,4,3); plot(t,x3),grid
title('Resposta projeto LQR para condicoes iniciais')
ylabel('state variable x3')
subplot(1,4,4); plot(t,x4),grid
title('Resposta projeto LQR para condicoes iniciais')
ylabel('state variable x4')
xlabel('t (sec)')

%% Filtro de Kalman

Xi=7e-4; % ruído aleatório na Eq. de estados
Theta=1e-8; % ruído aleatório na medição de saída
G = ss(A, [B, B], C, [D, D]);
[Gk,Ke,Pf]=kalman(G,Xi,Theta) % Filtro de Kalma
%P = eig(A-B*K)
%--- Entrar com a matriz de observabilidade N e testar seu posto
%N = [C' A'*C' ((A')^2)*C' ((A')^3)*C'];
%Ke = acker(A,B,P)'

AA_e = A-Ke*C-B*K;
BB_e = Ke;
CC_e = K;
DD_e = 0;
[num_e,den_e] = ss2tf(AA_e,BB_e,CC_e,DD_e)
AA_e

%% Modelo no espaço de estados para o projeto LQG para condições iniciais
sys_e = ss([A-B*K B*K; zeros(4,4) A-Ke*C],eye(8),eye(8),eye(8));


t = 0:0.01:20;
z = initial(sys_e,[1;0;0;0;1;0;0;0],t);
x1_ = [1 0 0 0 0 0 0 0]*z';
x2_ = [0 1 0 0 0 0 0 0]*z';
x3_ = [0 0 1 0 0 0 0 0]*z';
x4_ = [0 0 0 1 0 0 0 0]*z';
e1 = [0 0 0 0 1 0 0 0]*z';
e2 = [0 0 0 0 0 1 0 0]*z';
e3 = [0 0 0 0 0 0 1 0]*z';
e4 = [0 0 0 0 0 0 0 1]*z';
figure(2);
% title('Resposta projeto LQG a Condicao inicial')
% subplot(2,4,1); plot(t,x1), grid
% ylabel('state variable x1')
% title('Resposta projeto LQG a Condicao inicial')
% subplot(2,4,2); plot(t,x2),grid
% ylabel('state variable x2')
% title('Resposta projeto LQG a Condicao inicial')
% subplot(2,4,3); plot(t,x3),grid
% ylabel('state variable x3')
% title('Resposta projeto LQG a Condicao inicial')
% subplot(2,4,4); plot(t,x4),grid
% ylabel('state variable x4')
% title('Resposta projeto LQG a Condicao inicial')
% subplot(2,4,5); plot(t,e1),grid
% ylabel('error state variable e1')
% title('Resposta projeto LQG a Condicao inicial')
% subplot(2,4,6); plot(t,e2),grid
% ylabel('error state variable e2')
% title('Resposta projeto LQG a Condicao inicial')
% subplot(2,4,7); plot(t,e3),grid
% ylabel('error state variable e3')
% title('Resposta projeto LQG a Condicao inicial')
% subplot(2,4,8); plot(t,e4),grid
% ylabel('error state variable e4')
subplot(2,2,1); p=plot(t,e1),set(p,'Color','blue','LineWidth',2),grid
ylabel('error state variable e1')
title('Resposta projeto LQG a Condicao inicial')
subplot(2,2,2); p=plot(t,e2),set(p,'Color','blue','LineWidth',2),grid
ylabel('error state variable e2')
title('Resposta projeto LQG a Condicao inicial')
subplot(2,2,3); p=plot(t,e3),set(p,'Color','blue','LineWidth',2),grid
ylabel('error state variable e3')
title('Resposta projeto LQG a Condicao inicial')
subplot(2,2,4); p=plot(t,e4),set(p,'Color','blue','LineWidth',2),grid
ylabel('error state variable e4')
title('Resposta projeto LQG a Condicao inicial')
xlabel('t (sec)')

%% Plot da trajetoria real X trajetoria estimada
figure(3);
title('trajetoria real X trajetoria estimada')
subplot(1,4,1); p=plot(t,x1), set(p,'Color','blue','LineWidth',2),hold on, 
plot(t,x1_,'--r','LineWidth',2,...
                'MarkerSize',10), grid
legend('variavel de estado x1','estado estimado e1')
%title('Response to Initial Condition')
title('Resposta: trajetoria real X trajetoria estimada')
ylabel('state variable x1')

subplot(1,4,2); p=plot(t,x2), set(p,'Color','blue','LineWidth',2), hold on, 
plot(t,x2_,'--r','LineWidth',2,...
                'MarkerSize',10), grid
legend('variavel de estado x2','estado estimado e2')
%title('Response to Initial Condition')
title('trajetoria real X trajetoria estimada')
ylabel('state variable x2')

subplot(1,4,3); p=plot(t,x3), set(p,'Color','blue','LineWidth',2), hold on,
plot(t,x3_,'--r','LineWidth',2,...
                'MarkerSize',10),  grid
legend('variavel de estado x3','estado estimado e3')
%title('Response to Initial Condition')
title('trajetoria real X trajetoria estimada')
ylabel('state variable x3')

subplot(1,4,4); p=plot(t,x4), set(p,'Color','blue','LineWidth',2), hold on,
plot(t,x4_,'--r','LineWidth',2,...
                'MarkerSize',10),  grid
legend('variavel de estado x4','estado estimado e4')
%title('Response to Initial Condition')
title('trajetoria real X trajetoria estimada')
ylabel('state variable x4')
xlabel('t (sec)')



figure(4),
p=plot(t,x1), set(p,'Color','blue','LineWidth',2), hold on, 
plot(t,x1_,'--r','LineWidth',2,...
                'MarkerSize',10), grid
legend('variavel de estado x1','estado estimado e1')
%title('Response to Initial Condition')
title('trajetoria real X trajetoria estimada')
ylabel('state variable x2')


%%%%%%%%%%%%%%%%%%%%
%% Plote da resposta Real x resposta com LQG
figure(5);
%% Sistema inicial
[num,den] = ss2tf(A,B,C,D)
G = tf(num,den);

impulse(feedback(G,1),'b')
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
hold on

%% LQG
AA = A - Ke*C - B*K;
BB = Ke;
CC = K;
DD = D;
[numc,denc] = ss2tf(AA,BB,CC,DD)
GcG = tf(numc,denc)
Gcomp = GcG*G

% % Controlador LQG
% %Gc = [A-Ke'*C - B*K + (Ke*B*K)', Ke; K ,zeros(1,1)];
% Gamma = [   -1;
%             0;
%             0;
%             0]
% W = [jordan(Q),zeros(4,1); zeros(1,4),R]
% V = [Xi*Gamma*Gamma', zeros(4,1); zeros(1,4), Theta]
% % V é a matriz que é em fato a joint correlation function of signals ξ (t) and θ with
% % Xi is the covariance of ξ (t).
% [Af,Bf,Cf,Df] = lqg(A,B,C,D,W,V);
% Gc=ss(Af,Bf,Cf,Df)

impulse(feedback(ss(Gcomp),1),'r--')
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
hold on

%step(AA,BB,CC,DD)
legend('sistema sem compensacao','sistema com LQG')
grid on
title('Resposta do Sistema ao Impulso')


figure(6),
P = bodeoptions;
P.Grid = 'on';
P.PhaseUnits = 'deg';
P.FreqScale = 'log';
P.MagUnits = 'dB';
P.MagScale = 'log';
bodemag(G,'b')
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
hold on
%bode(feedback(GF,1),'r')
bodemag(Gcomp,'--r')
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
w=logspace(-2,2,4000); % Gera 200 pontos de 0.1 à 100
%plot(w,-3,'--') %linha para frequencia de corte
hold on
plot(0.958,-3,'*')
plot(1.258,-3,'*')
hold off
legend('sistema sem compensacao','sistema com LQG')
grid on

figure(7),
P = bodeoptions;
P.Grid = 'on';
P.PhaseUnits = 'deg';
P.FreqScale = 'log';
P.MagUnits = 'dB';
P.MagScale = 'log';

bodemag(G,'b')
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
hold on
bodemag(Gcomp,'r--')
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
w=logspace(-2,2,4000); % Gera 200 pontos de 0.1 à 100
%plot(w,-3,'--') %linha para frequencia de corte
grid on
legend('sistema sem compensacao','sistema com LQG')
%figure, bode(Sys,Gc*Sys,'g')

%title('Diagrama de Bode para Sistemas de Malha Fechada')
[Gm,Pm,Wcg,Wcp]=margin(Gcomp)
% Gm = margem de ganho
% Pm = Margem de fase
% Wcg = frequências critica de ganho 
% Wcp = frequências critica da fase 

%largura de banda
larg_banda_sys = bandwidth(G)
larg_banda_sys_lqg = bandwidth(Gcomp)

auto_valores_sys = eig(G) 
auto_valores_sys_lqg = eig(Gcomp)

[Wn,zeta,P] = damp(G)
[Wn_c,zeta_c,P_c] = damp(Gcomp)

%load_system('manipulador_LQG')
%open_system('manipulador_LQG')
%bdclose_all
% figure(7),
% plot(simout.Time,simout.Data)
% grid on
% hold on
% plot(simout1.Time,simout1.Data,'r')

%Uma vez que os autovalores do sistema compensado possui parte real mais deslocada para esquerda no semi-plano esquerdo complexo
% isso mostra que a resposta ao impulso do sistema compensado é mais rápido
% do que com o sistema sem o controlador. isso pode ser confirmado por meio
% banda passante do sistema compensado que é maior que do sistema sem
% compensação.