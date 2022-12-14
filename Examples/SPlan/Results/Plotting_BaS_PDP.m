% Plotting for BaS-PDP project
% Arthur Nascimento
% nascimento@gatech.edu
% Last updated Dec 13 2022

clear all; close all; clc;

% This is a plotting generator for BaS-PDP
% Make sure the files below are named correctly before being loaded

names = 0.4:0.1:1;
for j = 1:length(names)
    % Load files
    BaS(j) = load(['BaS_Cartpole_dt_0.12lim_', num2str(names(j)), '.mat']);
    BaS2(j) = load(['BaS_DumbWeights_dt_0.12lim_', num2str(names(j)), '.mat']);
    SPlan(j) = load(['SPlan_Cartpole_lim_', num2str(names(j)), '.mat']);
    
    % Declare variables
    X(j,:,:) = BaS(j).results.solved_trajectory;
    X2(j,:,:) = BaS2(j).results.solved_trajectory;
    Xsplan(j,:,:) = SPlan(j).results.solved_trajectory;
    cart_lim(j,:,:) = double(BaS(j).results.params.cart_lim);
    dt = double(BaS(1).results.params.dt);
    N = double(BaS(1).results.params.horizon);
    [~, ~, n] = size(X);
    
    % Compute BaS error
    T = 0:dt:N*dt;
    B = []; error = [];
    for i = 1:length(X(j))
        B(j,i) = 1/(cart_lim(j)^2 - X(j,i,1)^2);
        if n == 5
            error(j,i) = B(j,i) - X(j,i,5);
        end
    end
    
    % Plotting X1 --- SAFETY
    % BaS Dumb Weights
    figure(1)
    title('BaS with varying limits','Interpreter','latex', 'FontSize', 18);
    plot(T,X2(j,:,1)./cart_lim(j), 'LineWidth', 1.5,...
        'DisplayName', ['Limit = ' num2str(names(j))]);
    hold on; grid on;
    ylabel('% of track', 'FontSize', 18);
    ylim([-1.2 1.2])

    % Penalty (Safe PDP)
    figure(2)
    title('Penalty with varying limits','Interpreter','latex', 'FontSize', 18);
    plot(T,Xsplan(j,:,1)./cart_lim(j), 'LineWidth', 1.5,...
        'DisplayName', ['Limit = ' num2str(names(j))]);
    hold on; grid on;
    ylabel('% of track', 'FontSize', 18);
    ylim([-1.2 1.2])


    % Plotting X2 --- SUCCESS
    % BaS Dumb Weights
    figure(3)
    title('BaS with varying limits','Interpreter','latex', 'FontSize', 18);
    plot(T,X2(j,:,2), 'LineWidth', 1.5,...
        'DisplayName', ['Limit = ' num2str(names(j))]);
    hold on; grid on;

% % %     BaS better weights
% % %     figure(9999)
% % %     title('BaS with varying limits','Interpreter','latex', 'FontSize', 18);
% % %     plot(T,X(j,:,2), 'LineWidth', 1.5,...
% % %         'DisplayName', ['Limit = ' num2str(names(j))]);
% % %     hold on; grid on;

    % Penalty (Safe PDP)
    figure(4)
    title('Penalty with varying limits','Interpreter','latex', 'FontSize', 18);
    plot(T,Xsplan(j,:,2), 'LineWidth', 1.5,...
        'DisplayName', ['Limit = ' num2str(names(j))]);
    hold on; grid on;

end

% plot(T,zeros(size(T)),'--r','LineWidth',1);
figure(1)
plot(T, ones(length(T),1),'--k','LineWidth',0.8);
plot(T, -ones(length(T),1),'--k','LineWidth',0.8);
legend('limit = 0.4', 'limit = 0.5', 'limit = 0.6', 'limit = 0.7',...
    'limit = 0.8', 'limit = 0.9', 'limit = 1.0', 'barrier', 'FontSize', 14)

figure(2)
plot(T, ones(length(T),1),'--k','LineWidth',0.8);
plot(T, -ones(length(T),1),'--k','LineWidth',0.8);
legend('limit = 0.4', 'limit = 0.5', 'limit = 0.6', 'limit = 0.7',...
    'limit = 0.8', 'limit = 0.9', 'limit = 1.0', 'barrier', 'FontSize', 14)

figure(3)
plot(T, pi*ones(length(T),1),'--r','LineWidth',0.8);
legend('limit = 0.4', 'limit = 0.5', 'limit = 0.6', 'limit = 0.7',...
    'limit = 0.8', 'limit = 0.9', 'limit = 1.0', 'target', 'FontSize', 14)
ylabel('$\theta$','Interpreter','latex', 'FontSize', 18);

figure(4)
plot(T, pi*ones(length(T),1),'--r','LineWidth',0.8);
legend('limit = 0.4', 'limit = 0.5', 'limit = 0.6', 'limit = 0.7',...
    'limit = 0.8', 'limit = 0.9', 'limit = 1.0', 'target', 'FontSize', 14)
ylabel('$\theta$','Interpreter','latex', 'FontSize', 18);

%% Specific plots - final states

% A: lim = 1
figure(5)
subplot(2,2,1)
plot(T, X2(7,:,1),'LineWidth',1.5); hold on; grid on;
% plot(T, zeros(size(T)),'--r','LineWidth',1);
plot(T, cart_lim(7)*ones(length(T),1),'--k','LineWidth',0.8);
plot(T, -cart_lim(7)*ones(length(T),1),'--k','LineWidth',0.8);
ylabel('x (m)','Interpreter','latex', 'FontSize', 18);
ylim([-1.2*cart_lim(7) 1.2*cart_lim(7)])
title('X position (with BaS)', 'Interpreter', 'latex', 'FontSize', 18);

subplot(2,2,2)
plot(T, Xsplan(7,:,1),'LineWidth',1.5); hold on; grid on;
% plot(T, zeros(size(T)),'--r','LineWidth',1);
plot(T, cart_lim(7)*ones(length(T),1),'--k','LineWidth',0.8);
plot(T, -cart_lim(7)*ones(length(T),1),'--k','LineWidth',0.8);
ylabel('x (m)','Interpreter','latex', 'FontSize', 18);
ylim([-1.2*cart_lim(7) 1.2*cart_lim(7)])
title('X position (penalty method)', 'Interpreter', 'latex', 'FontSize', 18);

subplot(2,2,3)
plot(T, X2(7,:,2),'LineWidth',1.5); hold on; grid on;
plot(T, pi*ones(size(T)),'--r','LineWidth',1);
ylabel('$\theta$ (rad)','Interpreter','latex', 'FontSize', 18);
title('$\theta$ over time (with BaS)', 'Interpreter', 'latex', 'FontSize', 18);

subplot(2,2,4)
plot(T, Xsplan(7,:,2),'LineWidth',1.5); hold on; grid on;
plot(T, pi*ones(size(T)),'--r','LineWidth',1);
ylabel('$\theta$ (rad)','Interpreter','latex', 'FontSize', 18);
title('$\theta$ over time (penalty method)', 'Interpreter', 'latex', 'FontSize', 18);


% B: lim = 0.7
figure(6)
subplot(2,2,1)
plot(T, X2(4,:,1),'LineWidth',1.5); hold on; grid on;
% plot(T, zeros(size(T)),'--r','LineWidth',1);
plot(T, cart_lim(4)*ones(length(T),1),'--k','LineWidth',0.8);
plot(T, -cart_lim(4)*ones(length(T),1),'--k','LineWidth',0.8);
ylabel('x (m)','Interpreter','latex', 'FontSize', 18);
ylim([-1.2*cart_lim(4) 1.2*cart_lim(4)])
title('X position (with BaS)', 'Interpreter', 'latex', 'FontSize', 18);

subplot(2,2,2)
plot(T, Xsplan(4,:,1),'LineWidth',1.5); hold on; grid on;
% plot(T, zeros(size(T)),'--r','LineWidth',1);
plot(T, cart_lim(4)*ones(length(T),1),'--k','LineWidth',0.8);
plot(T, -cart_lim(4)*ones(length(T),1),'--k','LineWidth',0.8);
ylabel('x (m)','Interpreter','latex', 'FontSize', 18);
ylim([-1.2*cart_lim(4) 1.2*cart_lim(4)])
title('X position (penalty method)', 'Interpreter', 'latex', 'FontSize', 18);

subplot(2,2,3)
plot(T, X2(4,:,2),'LineWidth',1.5); hold on; grid on;
plot(T, pi*ones(size(T)),'--r','LineWidth',1);
ylabel('$\theta$ (rad)','Interpreter','latex', 'FontSize', 18);
title('$\theta$ over time (with BaS)', 'Interpreter', 'latex', 'FontSize', 18);

subplot(2,2,4)
plot(T, Xsplan(4,:,2),'LineWidth',1.5); hold on; grid on;
plot(T, pi*ones(size(T)),'--r','LineWidth',1);
ylabel('$\theta$ (rad)','Interpreter','latex', 'FontSize', 18);
title('$\theta$ over time (penalty method)', 'Interpreter', 'latex', 'FontSize', 18);


%% Plots (for single experiment)

single = load('BaS_Cartpole_dt_0.12.mat');
XX = single.results.solved_trajectory;

figure(7) 
subplot(2,2,1)
plot(T,XX(:,2),'LineWidth',1.5); hold on; grid on;
plot(T,pi*ones(size(T)),'--r','LineWidth',1);

subplot(2,2,2)
plot(T,XX(:,2),'LineWidth',1.5); hold on; grid on;
plot(T,pi*ones(size(T)),'--r','LineWidth',1);

subplot(2,2,3)
plot(T,XX(:,2),'LineWidth',1.5); hold on; grid on;
plot(T,pi*ones(size(T)),'--r','LineWidth',1);

subplot(2,2,4)
plot(T,XX(:,2),'LineWidth',1.5); hold on; grid on;
plot(T,pi*ones(size(T)),'--r','LineWidth',1);

%% FOR SINGLE EXPERIMENT --- DO NOT DELETE/REPLACE

% % % %% Single-file plots:
% % % %% Plot BaS
% % % if n == 5
% % %     figure(1)
% % %     plot(T, X(:,5), 'LineWidth', 1.5);
% % %     hold on; grid on;
% % %     plot(T, B, '--k', 'LineWidth', 1.5); plot(T, error, 'r', 'LineWidth', 1.5);
% % %     plot(T, zeros(1,length(T)), '--k', 'LineWidth', 1.5)
% % %     title(['dt = ' num2str(dt)],'Interpreter','latex', 'FontSize', 16);
% % %     ylabel('BaS magnitude','Interpreter','latex', 'FontSize', 16);
% % %     xlabel('Time','Interpreter','latex', 'FontSize', 16);
% % %     legend('Propagated', 'Expected', 'Error due to discretization',...
% % %         'Location', 'best', 'FontSize', 10);
% % % end
% % % 
% % % %% Plot final states
% % % figure(2) 
% % % title('States over time','Interpreter','latex', 'FontSize', 16);
% % % subplot(2,2,1)
% % % plot(T,X(:,1),'LineWidth',1.5); hold on; grid on;
% % % plot(T,zeros(size(T)),'--r','LineWidth',1);
% % % plot(T,cart_lim*ones(length(T),1),'--k','LineWidth',0.8);
% % % plot(T,-cart_lim*ones(length(T),1),'--k','LineWidth',0.8);
% % % ylabel('x (m)','Interpreter','latex', 'FontSize', 16);
% % % ylim([-1.2*cart_lim 1.2*cart_lim])
% % % 
% % % subplot(2,2,2)
% % % plot(T,X(:,2),'LineWidth',1.5); hold on; grid on;
% % % plot(T,pi*ones(size(T)),'--r','LineWidth',1);
% % % ylabel('$\theta$ (rad)','Interpreter','latex', 'FontSize', 16);
% % % 
% % % subplot(2,2,3)
% % % plot(T,X(:,3),'LineWidth',1.5); hold on; grid on;
% % % ylabel('$\dot{x}$ (m/s)','Interpreter','latex', 'FontSize', 16);
% % % 
% % % subplot(2,2,4)
% % % plot(T,X(:,4),'LineWidth',1.5); hold on; grid on;
% % % ylabel('$\dot{\theta}$ (rad/s)','Interpreter','latex', 'FontSize', 16);
% % % 
% % % % %% Plot intermediate trajectories --- OBSOLETE
% % % % 
% % % % for j = 1:10:n
% % % %     figure(3)
% % % %     plot(T, X_int(n, :, 1));
% % % %     hold on; grid on;
% % % % end
% % % % figure(3)
% % % % plot(T,X(:,1),'LineWidth',1.5);
% % % % plot(T,zeros(size(T)),'--r','LineWidth',1);
% % % % plot(T,cart_lim*ones(length(T),1),'--k','LineWidth',0.8);
% % % % plot(T,-cart_lim*ones(length(T),1),'--k','LineWidth',0.8);
% % % % ylim([-1.2*cart_lim 1.2*cart_lim])
