%% Single-file plots:
% Arthur Scaquetti do Nascimento
% nascimento@gatech.edu
% Last updated Dec 13 2022

clear all; close all; clc;

load('BaS_Cartpole_dt_0.001.mat');

% Declare variables
X = results.solved_trajectory;
cart_lim = double(results.params.cart_lim);
dt = double(results.params.dt);
N = double(results.params.horizon);
[~, ~, n] = size(X);

% Compute BaS error
T = 0:dt:N*dt;
B = []; error = [];
for i = 1:length(X)
    B(i) = 1/(cart_lim^2 - X(i,1)^2);
    if n == 5
        error(i) = B(i) - X(i,5);
    end
end

%% Plot BaS
if n == 5
    figure(1)
    plot(T, X(:,5), 'LineWidth', 1.5);
    hold on; grid on;
    plot(T, B, '--k', 'LineWidth', 1.5); plot(T, error, 'r', 'LineWidth', 1.5);
    plot(T, zeros(1,length(T)), '--k', 'LineWidth', 1.5)
    title(['dt = ' num2str(dt)],'Interpreter','latex', 'FontSize', 16);
    ylabel('BaS magnitude','Interpreter','latex', 'FontSize', 16);
    xlabel('Time','Interpreter','latex', 'FontSize', 16);
    legend('Propagated', 'Expected', 'Error due to discretization',...
        'Location', 'best', 'FontSize', 10);
end

%% Plot final states
figure(2) 
title('States over time','Interpreter','latex', 'FontSize', 16);
subplot(2,2,1)
plot(T,X(:,1),'LineWidth',1.5); hold on; grid on;
plot(T,zeros(size(T)),'--r','LineWidth',1);
plot(T,cart_lim*ones(length(T),1),'--k','LineWidth',0.8);
plot(T,-cart_lim*ones(length(T),1),'--k','LineWidth',0.8);
ylabel('x (m)','Interpreter','latex', 'FontSize', 16);
ylim([-1.2*cart_lim 1.2*cart_lim])
title('X (m)','Interpreter','latex', 'FontSize', 18);

subplot(2,2,2)
plot(T,X(:,2),'LineWidth',1.5); hold on; grid on;
plot(T,pi*ones(size(T)),'--r','LineWidth',1);
ylabel('$\theta$ (rad)','Interpreter','latex', 'FontSize', 16);
title('$\theta$ (rad)','Interpreter','latex', 'FontSize', 18);

subplot(2,2,3)
plot(T,X(:,3),'LineWidth',1.5); hold on; grid on;
ylabel('$\dot{x}$ (m/s)','Interpreter','latex', 'FontSize', 16);
title('$\dot{x}$ (m/s)','Interpreter','latex', 'FontSize', 18);

subplot(2,2,4)
plot(T,X(:,4),'LineWidth',1.5); hold on; grid on;
ylabel('$\dot{\theta}$ (rad/s)','Interpreter','latex', 'FontSize', 16);
title('$\dot{\theta}$ (rad/s)','Interpreter','latex', 'FontSize', 18);

% %% Plot intermediate trajectories --- OBSOLETE
% 
% for j = 1:10:n
%     figure(3)
%     plot(T, X_int(n, :, 1));
%     hold on; grid on;
% end
% figure(3)
% plot(T,X(:,1),'LineWidth',1.5);
% plot(T,zeros(size(T)),'--r','LineWidth',1);
% plot(T,cart_lim*ones(length(T),1),'--k','LineWidth',0.8);
% plot(T,-cart_lim*ones(length(T),1),'--k','LineWidth',0.8);
% ylim([-1.2*cart_lim 1.2*cart_lim])
