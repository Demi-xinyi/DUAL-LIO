%% 初始化环境
clear; close all; clc;
addpath(genpath('./src'));
addpath('./data');


%% 加载配置
config = config_default();

config.startTime = 0;
config.stopTime = 40;

% 设置数据路径
[filePathL, filePathR, truthPath] = deal( ...
    'data/fl', ...
    'data/rr', ...
    'data/truth.csv');

%% 加载数据
[timeL, accL, gyrL, timeR, accR, gyrR, groundtruth] = ...
    load_dataset(filePathL, filePathR, truthPath, config);


%% 运行
pos = run_lio(config,timeL, accL, gyrL, timeR, accR, gyrR);

%% 绘图对比轨迹


figure;
hold on;
plot3(groundtruth(:,1), groundtruth(:,2), groundtruth(:,3), 'r-', 'LineWidth', 2); hold on;
plot3(pos(:,1), pos(:,2), pos(:,3), 'LineWidth', 1.5);
legend('Ground Truth', 'Dual-IMU');
xlabel('X'); ylabel('Y'); zlabel('Z'); view(0,90); grid on;
title('Trajectory Comparison');
