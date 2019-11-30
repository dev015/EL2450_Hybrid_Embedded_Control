%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hybrid and Embedded control systems
% Homework 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
init_tanks;
g = 9.82;
Tau = 1/alpha1*sqrt(2*tank_h10/g);
k_tank = 60*beta*Tau;
gamma_tank = alpha1^2/alpha2^2;
uss = alpha2/beta*sqrt(2*g*tank_init_h2)*100/15; % steady state input
yss = 40; % steady state output

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Continuous Control design %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
numUT = k_tank; % Numerator of the upper tank
denUT = [Tau 1]; % Denominator of the upper tank
numLT = gamma_tank; % Numerator of the lower tank
denLT = [(gamma_tank*Tau) 1]; % Denominator of the lower tank

uppertank = tf(numUT,denUT); % Transfer function for upper tank
lowertank = tf(numLT,denLT); % Transfer function for lower tank
G = uppertank*lowertank; % Transfer function from input to lower tank level

% Calculating the PID parameters
% chi = 0.5;
% omega0 = 0.3147;
% zeta = 0.4705;
chi = 0.5;
zeta = 0.8;
omega0 = 0.2;

[K, Ti, Td, N] = polePlacePID(chi, omega0, zeta,Tau,gamma_tank,k_tank);
% PID transfer function
n1 = (K*Ti)*((N*Td)+1);
n2 = K*(1+(N*Ti));
n3 = K*N;
numF = [n1 n2 n3]; % Numerator of PID
denF = [Ti N*Ti 0]; % Denominator of PID
F = tf(numF,denF);

% sys = series(G,F);
% sys_cl = feedback(sys,1,-1);
% 
% TD = tf(1,1,'InputDelay',25);
% sys_CL = series(sys_cl,TD);
% 
% simout = sim('tanks_demo.mdl','SaveOutput','on','SaveTime','on','StopTime','150');
% plot(simout.H2)
% grid on
% hold on
% % figure
% opt = stepDataOptions('InputOffset',40,'StepAmplitude',10);
% step(sys_CL,opt)
% hold off
% [Gm,Pm,Wcg,Wcp] = margin(sys);
% margin(sys)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% Digital Control design %%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discretize the continous controller, save it in state space form
% Consider the Ts = 0.04
% Ts = 0.04;
% [A_f,B_f,C_f,D_f] = tf2ss(numF,denF);
% sysd = ss(A_f,B_f,C_f,D_f);
% sysD = c2d(sysd,Ts,'zoh');
% A_discretized = sysD.A;
% B_discretized = sysD.B;
% C_discretized = sysD.C;
% D_discretized = sysD.D;

%h = [10 5 1 0.1 0.04 0.01 0.005]; % Sampling time
%h = [0.1 0.2 0.3 0.4 0.5 2 4]
%h = [1 1.2 1.5 1.8 2 2.5 3]
%for i = 1:7
    figure
    %Ts = h(i);
    Ts = 4;
    [A_f,B_f,C_f,D_f] = tf2ss(numF,denF);
    sysd = ss(A_f,B_f,C_f,D_f);
    sysD = c2d(sysd,Ts,'zoh');
    A_discretized = sysD.A;
    B_discretized = sysD.B;
    C_discretized = sysD.C;
    D_discretized = sysD.D;
    
%     simout_cont = sim('tanks_demo_1.mdl','SaveOutput','on','SaveTime','on','StopTime','150');
%     hold on 
%     plot(simout_cont.h2)
    simout_dis = sim('tanks.mdl','SaveOutput','on','SaveTime','on','StopTime','150');
    plot(simout_dis.h2)
    hold on
    %legend('Cont+ZOH', 'Discrete')
    %grid on
%     title (['System Response for Ts = ',num2str(h(i))])
%     xlabel ('time (sec)')
%     ylabel ('Output (h2)')
%     hold off
%end
% legendCell = cellstr(num2str(h', 'Ts=%-d'));
% legend(legendCell);
% grid on
% title ('Comparing the control performance between Cont+ZOH and Discretized controller')
% xlabel ('time (sec)')
% ylabel ('Output (h2)')
% hold off
%figure(1)
%Ts = 4;
% simout = sim('tanks.mdl','SaveOutput','on','SaveTime','on','StopTime','150');
% plot(simout.h2)
% grid on
% title ('System Response (Task 11)for Ts = 4 sec')
% xlabel ('time (sec)')
% ylabel ('Output (h2)')
% hold on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Discrete Control design %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discretize the continous state space system, save it in state space form
% [Phi,Gamma,C,D] = ?
% Consider Ts = 4s
 Ts = 4;
%[A_p,B_p,C_p,D_p] = tf2ss(2.187,[154.5 24.86 1]);
A_p = [-1/Tau 0; 1/Tau -1/(gamma_tank*Tau)];
B_p = [k_tank/Tau;0];
C_p = [0 1];
D_p = 0;
sysp = ss(A_p,B_p,C_p,D_p);
sysDis = c2d(sysp,Ts,'zoh');
Phi = sysDis.A;
Gamma = sysDis.B;
C = sysDis.C;
D = sysDis.D;

% % Observability and reachability
% Wo = obsv(Phi,C);
% isObsv = length(Phi) - rank(Wo);
% Wc = ctrb(Phi,Gamma);
% isCtrb = length(Phi) - rank(Wc);
% %% %%%%%%%%%%%%%% Task 15 %%%%%%%%%%%%%%%% %%
% % State feedback controller gain
% % Using Ackermann Formula
% p = [0.5 -0.3];
% L = acker(Phi,Gamma,p);
% % observer gain
% pO = [-0.6, -0.7];
% M = acker(Phi',C',pO);
% 
% %% %%%%%%%%%%%%% Task 17 %%%%%%%%%%%%%%%% %%
p_controller = [0.4675+0.2436i 0.4675-0.2436i];
p_observer = [0.1261 0.1453];
L = acker(Phi,Gamma,p_controller);
M = acker(Phi',C',p_observer);

% reference gain
num = 1;
den1 = C/(eye(2)-Phi+(Gamma*L));
den = den1 * Gamma;
lr = num / den;
% augmented system matrices
Aa = [Phi -(Gamma*L); M'*C Phi-(Gamma*L)-(M'*C)];
Ba = [Gamma.*lr; Gamma.*lr];


simout = sim('tanks_task18.mdl','SaveOutput','on','SaveTime','on','StopTime','150');
plot(simout.h2)
grid on
title ('System Response for Ts = 4 sec')
legend('Discretized','Discrete Designed')
xlabel ('time (sec)')
ylabel ('Output (h2)')
hold off
% 
% %% Quantization BLock %%
% qt = [12.5 6.25 1.56 0.78125 0.39 0.097] %[3,4,6,7,8,10]
% figure
% for i = 1:6
%     Qt = qt(i)
%     simout = sim('tanks_demo_2.mdl','SaveOutput','on','SaveTime','on','StopTime','200');
%     plot(simout.h2)
%     hold on
% end
% legendCell = cellstr(num2str(qt', 'Quantization Level=%-d'));
% legend(legendCell)
% legend('Location','south')
% title('System Response for different Quantization Levels')
% xlabel('time')
% ylabel('Output(h2)')
% grid on
% hold off