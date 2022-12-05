% Plots for BaS-PDP poster
% Arthur Nascimento
% AE 8750 Fall 2022

clf; clc; clear all;

BaS = load('BaS_Cartpole_Testing_lim_1.mat');
SPlan = load('SPlan_Cartpole_Arthur_lim_1.mat');

names = [[0.4: 0.1: 0.8] 1];
for j = 1:length(names)
    mBaS(j) = load(['BaS_Cartpole_Testing_lim_', num2str(names(j)), '.mat']);
    
    if names(j) == 0.91
        names(j) = 0.9;
    end

    mSPlan(j) = load(['SPlan_Cartpole_Arthur_lim_', num2str(names(j)), '.mat']);
end

cart_limit = double(BaS.results.cart_lim);

Xbas = BaS.results.solved_trajectory;
Xsoft = SPlan.results.solved_trajectory;

bas = 1./(BaS.results.barrier_function);     % CUIDADOOOOO
splan_barrier = 1./(SPlan.results.inverse_BaS);

zero = zeros(length(Xbas));
pis = pi.*ones(length(Xbas));
T = 0:0.12:0.12*30;

safety = []; safesum = 0;
for i = 1:length(splan_barrier)
    if xor(max(Xsoft(:,1)) > cart_limit, min(max(Xsoft(:,1))) < -cart_limit)
        safety(i) = 1;
    else
        safety(i) = 0;
    end
    safesum = safesum + safety(i);
end

disp('Safety at the end: (>0 = not safe)')
safety(end)

figure(1)
subplot(2,1,1)
plot(T,Xbas(:,1),'LineWidth',1.5); hold on; grid on;
plot(T,zero,'--r','LineWidth',1);
plot(T,cart_limit*ones(length(T),1),'--k','LineWidth',2);
plot(T,-cart_limit*ones(length(T),1),'--k','LineWidth',2);
title('Cart position vs time','Interpreter','latex');
ylabel('$x$ (m)','Interpreter','latex');
xlabel('Time (s)','Interpreter','latex');
legend('x', '$x_{target}$','Interpreter','latex');
ylim([-cart_limit-0.1 cart_limit+0.1])

% % % % % % subplot(2,2,2)
% % % % % % plot(T,Xsoft(:,1),'LineWidth',1.5); hold on; grid on;
% % % % % % plot(T,zero,'--r','LineWidth',1);
% % % % % % plot(T,cart_limit*ones(length(T),1),'--k','LineWidth',2);
% % % % % % plot(T,-cart_limit*ones(length(T),1),'--k','LineWidth',1.5);
% % % % % % title('Cart position vs time','Interpreter','latex');
% % % % % % ylabel('$x$ (m)','Interpreter','latex');
% % % % % % xlabel('Time (s)','Interpreter','latex');
% % % % % % legend('x', '$x_{target}$','Interpreter','latex');
% % % % % % ylim([-cart_limit-0.1 cart_limit+0.1])

subplot(2,1,2)
plot(T,bas,'LineWidth',1); grid on
title('BaS vs time','Interpreter','latex');
ylabel('Barrier state $z$','Interpreter','latex');
xlabel('Time (s)','Interpreter','latex');
box on;

% % % % % subplot(2,2,4)
% % % % % plot(T,bas,'LineWidth',1); grid on
% % % % % title('BaS vs time','Interpreter','latex');
% % % % % ylabel('Barrier state $z$','Interpreter','latex');
% % % % % xlabel('Time (s)','Interpreter','latex');
% % % % % box on;

iter = [1:1:1500];
%%
figure(2)
hold on; grid on;
subplot(1,2,1)
hold on; grid on;
title('Soft-constrained PDP','Interpreter','latex');
ylabel('$Loss$','Interpreter','latex');
xlabel('Iteration','Interpreter','latex');
subplot(1,2,2)
hold on; grid on;
title('BaS-PDP','Interpreter','latex');
ylabel('$Loss$','Interpreter','latex');
xlabel('Iteration','Interpreter','latex');
for k = 1:length(mSPlan)
    txt = ['Limit = ',num2str(names(k))];
    loss_splan(k,:) = mSPlan(k).results.loss_trace;
    loss_bas(k,:) = mBaS(k).results.loss_barrier_trace;
    subplot(1,2,1)
    plot(iter, loss_splan(k,:),'LineWidth', 1.5,'DisplayName',txt)
    subplot(1,2,2)
    plot(iter, loss_bas(k,:),'LineWidth', 1.5,'DisplayName',txt)
end
% % % % % title('Soft-constrained PDP','Interpreter','latex');
% % % % % ylabel('$Loss$','Interpreter','latex');
% % % % % xlabel('Iteration','Interpreter','latex');
hold off
legend show
subplot(1,2,1)
hold off
legend show

% % % % % % % % % % % figure(3)
% % % % % % % % % % % hold on; grid on;
% % % % % % % % % % % for kk = 1:length(mBaS)
% % % % % % % % % % %     txt = ['Limit = ',num2str(names(kk))];
% % % % % % % % % % %     loss_bas(kk,:) = mBaS(kk).results.loss_barrier_trace;
% % % % % % % % % % %     plot(iter, loss_bas(kk,:),'LineWidth', 1.5,'DisplayName',txt)
% % % % % % % % % % % end
% % % % % % % % % % % title('BaS-PDP','Interpreter','latex');
% % % % % % % % % % % ylabel('$Loss$','Interpreter','latex');
% % % % % % % % % % % xlabel('Iteration','Interpreter','latex');
% % % % % % % % % % % hold off
% % % % % % % % % % % legend show
