%% UNIVERSIDADE ESTADUAL DO MARANHÃO
%% CURSO DE ENGENHARIA DA COMPUTAÇÃO
%% TRABALHO DE CONCLUSÃO DE CURSO
%% TÍTULO: PROJETO DE UM FILTRO DE KALMAN EM UM MANIPULADOR ROBÓTICO USANDO UM ALGORITMO GENÉTICO

%%              CÓDIGO 2: BARREIRAS DE DESEMPENHO E ESTABILIDADE ROBUSTAS


Aux = ss(Gcomp)
A = Aux.a; B = Aux.b; C = Aux.c; D=Aux.d;
pause;
sysG=ss(A,B,C,D);
w=logspace(-2,3,100);

alfar = 20*log10(0.10);
alfad = 20*log10(0.10);
alfas = 20*log10(0.15);

aux=[0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0];
b=0;
w_=i*w;
%Ga = wgn(2,2,1,'linear');
%den = ss2tf(Ga,eye(1,2)',eye(1,2),eye(1,1));
for j=1:100,
    inverro(j)= 100000;
    for i=1:10
        
        %gr=tf(1/65, den);
        gr=tf(625, [1 50*aux(i) 625]);
        %gr=tf(9, [1 6*aux(i) 9]);
        sysGr=series(gr,eye(1));
        sysGR=series(sysG,sysGr);
        [AR,BR,CR,DR]=ssdata(sysGR);

        G = C*inv(w_(j)*eye(size(A))-A)*B-D;
        GR = CR*inv(w_(j)*eye(size(AR))-AR)*BR-DR;
        E = (GR - G)*inv(G);
        S = svd(E);
        aux3= 1/(S(1));
        if aux3 <= inverro(j),
            inverro(j) = aux3;
        end;

    end;

end;
figure(8)
semilogx(w,20*log10(inverro),'r');
hold on;


w=linspace(0.01,0.5,1000);
semilogx(w,alfar,'r');
t=linspace(-50,alfar);
semilogx(0.5,t,'r');

w=linspace(0.01,0.7,1000);
semilogx(w,alfas,'r');
t=linspace(-50,alfas);
semilogx(0.7,t,'r');

grid;
title('Performance and stability barriers');
xlabel('frequ�ncia');
ylabel('dB');

save barreira inverro


% %Determina��o da barreira de estabilidade e da barreira de desempenho 
figure(9); 
alfar=20*log10(0.10);  %sinal de refer�ncia
alfad=20*log10(0.10);  %rejei��o de perturba��es
alfas=20*log10(0.15);  %sensibilidade a varia��es da planta
w=linspace(0.01,0.5,1000);
semilogx(w,alfar,'r--*');%barreira do desempenho
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
d1 = semilogx(w,alfar,'r--*');
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
hold on;
t=linspace(-50,alfar);
semilogx(0.5,t,'r--*');
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
w=linspace(0.01,0.7,1000);
semilogx(w,alfas,'r--*');%barreira do desempenho
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
t=linspace(-50,alfas);
semilogx(0.7,t,'r--*');
h = findobj(gcf,'type','line');
set(h,'linewidth',2);

sysG=ss(A,B,C,D);
w=logspace(-2,3,100);
[sv,w]=sigma(sysG,w);   %valores singulares da planta nominal.
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
semilogx(w,20*log10(sv));

load barreira inverro;  %inverso do erro 
semilogx(w,20*log10(inverro),'black--*');%barreira da estabilidade
h = findobj(gcf,'type','line');
set(h,'linewidth',2.5);
e = semilogx(w,20*log10(inverro),'black--*');%barreira da estabilidade
title('System with LQG and performance barriers and stability');
xlabel('frequ�ncia');
ylabel('dB');
%legend('Desempenho robusto','Sistema com LQG','Estabilidade Robusta')
grid on;