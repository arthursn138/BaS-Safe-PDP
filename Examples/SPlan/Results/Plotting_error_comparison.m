% Plotting for BaS-PDP project
% Arthur Nascimento
% nascimento@gatech.edu
% Last updated Dec 9 2022

clear all; close all; clc;

% This is a plotting generator for BaS-PDP
% Make sure the file(s) to be loaded is(are) correct

%% Multiple files
% Data loading and trajectories extraction
data12 = load('BaS_Cartpole_dt_0.12.mat');
data05 = load('BaS_Cartpole_dt_0.05.mat');
data02 = load('BaS_Cartpole_dt_0.02.mat');
data01 = load('BaS_Cartpole_dt_0.01.mat');
data005 = load('BaS_Cartpole_dt_0.005.mat');
data002 = load('BaS_Cartpole_dt_0.002.mat');
data001 = load('BaS_Cartpole_dt_0.001.mat');
cart_lim = 1;

X_12 = data12.results.solved_trajectory;
X_05 = data05.results.solved_trajectory;
X_02 = data02.results.solved_trajectory;
X_01 = data01.results.solved_trajectory;
X_005 = data005.results.solved_trajectory;
X_002 = data002.results.solved_trajectory;
X_001 = data001.results.solved_trajectory;

T_12 = 0:0.12:3; T_05 = 0:0.05:3; T_02 = 0:0.02:3; T_01 = 0:0.01:3;
T_005 = 0:0.005:3; T_002 = 0:0.002:3; T_001 = 0:0.001:3;
e_12 = []; e_05 = []; e_02 = []; e_01 = [];
e_005 = []; e_002 = []; e_001 = [];
for i=1:length(X_12)
    e_12(i) = (1/(cart_lim^2 - X_12(i,1)^2)) - X_12(i,5);
end

for i=1:length(X_05)
    e_05(i) = (1/(cart_lim^2 - X_05(i,1)^2)) - X_05(i,5);
end
for i=1:length(X_02)
    e_02(i) = (1/(cart_lim^2 - X_02(i,1)^2)) - X_02(i,5);
end
for i=1:length(X_01)
    e_01(i) = (1/(cart_lim^2 - X_01(i,1)^2)) - X_01(i,5);
end
for i=1:length(X_005)
    e_005(i) = (1/(cart_lim^2 - X_005(i,1)^2)) - X_005(i,5);
end
for i=1:length(X_002)
    e_002(i) = (1/(cart_lim^2 - X_002(i,1)^2)) - X_002(i,5);
end
for i=1:length(X_001)
    e_001(i) = (1/(cart_lim^2 - X_001(i,1)^2)) - X_001(i,5);
end

figure
semilogy(T_12, e_12, 'LineWidth', 1.5); hold on; grid on;
plot(T_05, e_05, 'LineWidth', 1.5); 
plot(T_02, e_02, 'LineWidth', 1.5); 
plot(T_01, e_01, 'LineWidth', 1.5); 
plot(T_005, e_005, 'LineWidth', 1.5); 
plot(T_002, e_002, 'LineWidth', 1.5); 
plot(T_001, e_001, 'LineWidth', 1.5);
legend('$dt = 0.12$', '$dt = 0.05$', '$dt = 0.02$', '$dt = 0.01$',...
    '$dt = 0.005$', '$dt = 0.002$', '$dt = 0.001$', 'Interpreter',...
    'latex', 'FontSize', 12, 'Location', 'best');
xlabel('Time','Interpreter','latex', 'FontSize', 16)
ylabel('Error magnitude','Interpreter','latex', 'FontSize', 16);

% datas = [12 05 02 01 005 002 001];
% for i=1:length(datas)
%     Xbas = dataresults.solved_trajectory;
% end

