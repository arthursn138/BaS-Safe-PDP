% Plotting for BaS-PDP project

clear all; close all; clc;

% load('BaS_Cartpole_Testing_lim_1.mat')
load('ALTRO_BaS_Cartpole_Testing_lim_1.mat')

Xopt = results.coc_sol.state_traj_opt;

% dt = double(results.dt);
% N = double(results.horizon);
% cart_lim = double(results.cart_lim);

cart_lim = 1;
T = [0:1:150];

for i=1:length(Xopt)
    B(i) = 1/(cart_lim^2 - Xopt(i,1));
    error(i) = B(i) - Xopt(i,5);
end

% BAS
figure(1)
plot(T, Xopt(:,5), 'LineWidth', 1.5);
hold on; grid on;
plot(T, B, 'LineWidth', 1.5); plot(T, error, 'LineWidth', 1.5); 
plot(T, zeros(1,length(T)), '--k', 'LineWidth', 1.5)
title('BaS with ALTRO','Interpreter','latex');
ylabel('(m)','Interpreter','latex');
xlabel('Time (s)','Interpreter','latex');
legend('$X_5$', '$\frac{1}{h(x)}$', 'Difference ($X_5 - \frac{1}{h(x)}$)', 'Interpreter','latex');

% STATES
figure(2)
subplot(2,2,1)
plot(T,Xopt(:,1),'LineWidth',1.5); hold on; grid on;
plot(T,zeros(size(T)),'--r','LineWidth',1);
plot(T,cart_lim*ones(length(T),1),'--k','LineWidth',2);
plot(T,-cart_lim*ones(length(T),1),'--k','LineWidth',2);
% title('Cart position vs time','Interpreter','latex');
% ylabel('$x$ (m)','Interpreter','latex');
% xlabel('Time (s)','Interpreter','latex');
% legend('x', '$x_{target}$','Interpreter','latex');
% ylim([-cart_lim-0.1 cart_lim+0.1])

subplot(2,2,2)
plot(T,Xopt(:,2),'LineWidth',1.5); hold on; grid on;
plot(T,pi*ones(size(T)),'--r','LineWidth',1);

subplot(2,2,3)
plot(T,Xopt(:,3),'LineWidth',1.5); hold on; grid on;

subplot(2,2,4)
plot(T,Xopt(:,4),'LineWidth',1.5); hold on; grid on;