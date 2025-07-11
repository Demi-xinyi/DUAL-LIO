clear;
close all;
clc;
addpath('Quaternions');
addpath('ximu_matlab_library');

%groundtruth
% 读取CSV文件，跳过标题行
% % data = readmatrix('gazebo_gt.csv');
data = readmatrix('truth_ys.csv');
mocap_euler = readmatrix('mocap_euler_ys.csv');

% 提取数据
time = [1:size(data)];
positionx = data(:, 1);
positiony = data(:, 2);
positionz = data(:, 3);
groundtruth=[positionx,positiony,positionz];
% orientationx = data(:, 9);
% orientationy = data(:, 10);
% orientationz = data(:, 11);

% Select dataset (comment in/out)
% % 
% filePathl = 'Datasets/gazebo_lffoot_record';
% startTimel = 0;
% stopTimel = 65;
% 
% filePathr = 'Datasets/gazebo_rhfoot_record';
% startTimer = 0;
% stopTimer = 65;
filePathl = 'Datasets/ysfl';
startTimel = 0;
stopTimel = 46.471;

filePathr = 'Datasets/ysrr';
startTimer = 0;
stopTimer = 46.471;
% filePathl = 'Datasets/square3m3mdual1';
% startTimel = 0;
% stopTimel = 63;
% 
% filePathr = 'Datasets/square3m3mdual2';
% startTimer = 0;
% stopTimer = 63;

% filePathl = 'Datasets/a1';
% startTimel = 0;
% stopTimel = 63;
% 
% filePathr = 'Datasets/a1';
% startTimer = 0;
% stopTimer = 63;

% filePathl = 'Datasets/chuangxindashaimu1';
% startTimel = 0;
% stopTimel = 617;
% 
% filePathr = 'Datasets/chuangxindashaimu2';
% startTimer = 0;
% stopTimer = 617;
% -------------------------------------------------------------------------
% Import data
samplePeriod = 1/200;
xIMUdata = xIMUdataClass(filePathl, 'InertialMagneticSampleRate', 1/samplePeriod);
timel = xIMUdata.CalInertialAndMagneticData.Time;
gyrXl = xIMUdata.CalInertialAndMagneticData.Gyroscope.X;
gyrYl = xIMUdata.CalInertialAndMagneticData.Gyroscope.Y;
gyrZl = xIMUdata.CalInertialAndMagneticData.Gyroscope.Z;
accXl = xIMUdata.CalInertialAndMagneticData.Accelerometer.X;
accYl = xIMUdata.CalInertialAndMagneticData.Accelerometer.Y;
accZl = xIMUdata.CalInertialAndMagneticData.Accelerometer.Z;
clear('xIMUdata');

% Import data
xIMUdata = xIMUdataClass(filePathr, 'InertialMagneticSampleRate', 1/samplePeriod);
timer = xIMUdata.CalInertialAndMagneticData.Time;
gyrXr = xIMUdata.CalInertialAndMagneticData.Gyroscope.X;
gyrYr = xIMUdata.CalInertialAndMagneticData.Gyroscope.Y;
gyrZr = xIMUdata.CalInertialAndMagneticData.Gyroscope.Z;
accXr = xIMUdata.CalInertialAndMagneticData.Accelerometer.X;
accYr = xIMUdata.CalInertialAndMagneticData.Accelerometer.Y;
accZr = xIMUdata.CalInertialAndMagneticData.Accelerometer.Z;
clear('xIMUdata');


% -------------------------------------------------------------------------
% Manually frame data

% startTime = 0;
% stopTime = 10;

indexSel = find(sign(timel-startTimel)+1, 1) : find(sign(timel-stopTimel)+1, 1);
%sign大于0的值返回1，小于0的值返回-1，等于0的返回0。
%结果加1，使得在"startTime"前的时间点返回0，而在"startTime"之后的时间点返回2。
%k = find(X,n) 返回与 X 中的非零元素对应的前 n 个索引。
timel = timel(indexSel);
gyrXl = gyrXl(indexSel, :);
gyrYl = gyrYl(indexSel, :);
gyrZl = gyrZl(indexSel, :);
accXl = accXl(indexSel, :);
accYl = accYl(indexSel, :);
accZl = accZl(indexSel, :);

indexSel = find(sign(timer-startTimer)+1, 1) : find(sign(timer-stopTimer)+1, 1);
timer = timer(indexSel);
gyrXr = gyrXr(indexSel, :);
gyrYr = gyrYr(indexSel, :);
gyrZr = gyrZr(indexSel, :);
accXr = accXr(indexSel, :);
accYr = accYr(indexSel, :);
accZr = accZr(indexSel, :);

start_plot = 1;
end_plot = length(mocap_euler);
timel = timel(start_plot:end_plot);
gyrXl = gyrXl(start_plot:end_plot, :);
gyrYl = gyrYl(start_plot:end_plot, :);
gyrZl = gyrZl(start_plot:end_plot, :);
accXl = accXl(start_plot:end_plot, :);
accYl = accYl(start_plot:end_plot, :);
accZl = accZl(start_plot:end_plot, :);
timer = timer(start_plot:end_plot);
gyrXr = gyrXr(start_plot:end_plot, :);
gyrYr = gyrYr(start_plot:end_plot, :);
gyrZr = gyrZr(start_plot:end_plot, :);
accXr = accXr(start_plot:end_plot, :);
accYr = accYr(start_plot:end_plot, :);
accZr = accZr(start_plot:end_plot, :);
%------------------------------------------------------------
%%LP
% filtCutOff = 1;
% [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
% gyrX = filtfilt(b, a, gyrX);
% gyrY = filtfilt(b, a, gyrY);
% gyrZ = filtfilt(b, a, gyrZ);
% accX = filtfilt(b, a, accX);
% accY = filtfilt(b, a, accY);
% accZ = filtfilt(b, a, accZ);
% -------------------------------------------------------------------------
% Detect stationary periods

% Compute accelerometer magnitude
% acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);

% acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);
% HP filter accelerometer data
% filtCutOff = 8;
% [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
% acc_magFilt = filtfilt(b, a, acc_mag);
% 
% % Compute absolute value 绝对值
% acc_magFilt = abs(acc_magFilt);
% 
% % LP filter accelerometer data
% filtCutOff = 14;
% [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
% acc_magFilt = filtfilt(b, a, acc_magFilt);
% 
% % Threshold detection 阈值检测
% stationary = acc_magFilt <0.1;
% -------------------------------------------------------------------------
%% 用角速度判断
% Detect stationary periods

gyr_magl = sqrt(gyrXl.*gyrXl + gyrYl.*gyrYl + gyrZl.*gyrZl);
%HP filter accelerometer data
filtCutOff = 10;%12
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
gyr_magFiltl = filtfilt(b, a, gyr_magl);

% Compute absolute value 绝对值
gyr_magFiltl = abs(gyr_magFiltl);

% LP filter accelerometer data
filtCutOff = 11;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
gyr_magFiltl = filtfilt(b, a, gyr_magFiltl);

% Threshold detection 阈值检测
stationaryl = gyr_magFiltl <0.038;

gyr_magr = sqrt(gyrXr.*gyrXr + gyrYr.*gyrYr + gyrZr.*gyrZr);
%HP filter accelerometer data
filtCutOff = 10;%12
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
gyr_magFiltr = filtfilt(b, a, gyr_magr);

% Compute absolute value 绝对值
gyr_magFiltr = abs(gyr_magFiltr);

% LP filter accelerometer data
filtCutOff = 11;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
gyr_magFiltr = filtfilt(b, a, gyr_magFiltr);

% Threshold detection 阈值检测
stationaryr = gyr_magFiltr <0.038;
% figure;
% plot(0.1*stationaryr);
% hold on;
% plot(gyr_magFiltr);
% % plot(stationaryl,'b');
% % plot(gyr_magFiltl,'b');
% hold off ; 
% figure;
% hold on;
% plot(stationaryr,'r');
% 
% plot(gyr_magFiltr,'r');
% plot(0.1*stationaryl);
% plot(gyr_magFiltl);
% hold off ; 

% figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Sensor Data');
%'Position' - 可绘制区域的位置和大小[left bottom width height]
%可绘制区域的位置和大小，指定为 [left bottom width height] 形式的向量。
% 指定 Name 属性为Sensor Data。NumberTitle 属性设置为 'off'，生成的标题不包含图窗编号。
% ax(1) = subplot(2,1,1);
%     hold on;
%     plot(timel, gyrXl, 'r');
%     plot(timel, gyrYl, 'g');
%     plot(timel, gyrZl, 'b');
%     plot(timel, gyr_magFiltl, ':k');
%     plot(timel, stationaryl, 'k', 'LineWidth', 4);
%     title('Gyroscopel');
%     xlabel('Time (s)');
%     ylabel('Angular velocityl (^\circ/s)');
%     legend('X', 'Y', 'Z','stationary');
%     hold off;
% ax(2) = subplot(2,1,2);
%     hold on;
%     plot(timel, accXl, 'r');
%     plot(timel, accYl, 'g');
%     plot(timel, accZl, 'b');
% %     plot(time, acc_magFilt, ':k');
% 
%     plot(timel, stationaryl, 'k', 'LineWidth', 2);
%     title('Accelerometerl');
%     xlabel('Time (s)');
%     ylabel('Accelerationl (g)');
%     legend('X', 'Y', 'Z', 'Filtered', 'Stationary');
%     hold off;
%     linkaxes(ax,'x');%linkaxes(ax,dimension) 同步指定坐标区维度的坐标区范围。例如，linkaxes(ax,'x') 仅同步 x 轴的范围。
% 
%     % Plot data raw sensor data and stationary periods
% 
% figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Sensor Data');
% %'Position' - 可绘制区域的位置和大小[left bottom width height]
% %可绘制区域的位置和大小，指定为 [left bottom width height] 形式的向量。
% % 指定 Name 属性为Sensor Data。NumberTitle 属性设置为 'off'，生成的标题不包含图窗编号。
% ax(1) = subplot(2,1,1);
%     hold on;
%     plot(timer, gyrXr, 'r');
%     plot(timer, gyrYr, 'g');
%     plot(timer, gyrZr, 'b');
%     plot(timer, gyr_magFiltr, ':k');
%      plot(timer, stationaryr, 'k', 'LineWidth', 2);
%     title('Gyroscoper');
%     xlabel('Time (s)');
%     ylabel('Angular velocityr (^\circ/s)');
%     legend('X', 'Y', 'Z','stationary');
%     hold off;
% ax(2) = subplot(2,1,2);
%     hold on;
%     plot(timer, accXr, 'r');
%     plot(timer, accYr, 'g');
%     plot(timer, accZr, 'b');
% %     plot(time, acc_magFilt, ':k');
% 
%     plot(timer, stationaryr, 'k', 'LineWidth', 2);
%     title('Accelerometerr');
%     xlabel('Time (s)');
%     ylabel('Accelerationr (g)');
%     legend('X', 'Y', 'Z', 'Filtered', 'Stationary');
%     hold off;
%     linkaxes(ax,'x');%linkaxes(ax,dimension) 同步指定坐标区维度的坐标区范围。例如，linkaxes(ax,'x') 仅同步 x 轴的范围。

%     %%注意单位
gyrXl = rad2deg(gyrXl);
gyrYl = rad2deg(gyrYl);
gyrZl = rad2deg(gyrZl);
accXl = accXl /9.81;
accYl = accYl /9.81;
accZl = accZl /9.81;

gyrXr = rad2deg(gyrXr);
gyrYr = rad2deg(gyrYr);
gyrZr = rad2deg(gyrZr);
accXr = accXr /9.81;
accYr = accYr /9.81;
accZr = accZr /9.81;
% -------------------------------------------------------------------------
% Compute orientation
% for 
start_exponent = -12;
end_exponent = 8;
param = logspace(start_exponent, end_exponent, end_exponent - start_exponent + 1);
param = 1e-5;
flag = length(param);
rmse = zeros(flag+1,2);
for j = 1:flag

AHRSalgorithml = AHRS('SamplePeriod', 1/200, 'Kp',0.1, 'KpInit', 1);%原本kp=1，ki=1
AHRSalgorithmr = AHRS('SamplePeriod', 1/200, 'Kp', 0.01, 'KpInit', 0);%原本kp=1，ki=1

% Initial convergence
initPeriod = 2;
indexSell = 1 : find(sign(timel-(timel(1)+initPeriod))+1, 1);
%找出前两秒钟的索引

% Initial convergence
indexSelr = 1 : find(sign(timer-(timer(1)+initPeriod))+1, 1);
%找出前两秒钟的索引

for i = 1:2000
    AHRSalgorithml.UpdateIMU([0 0 0], [mean(accXl(indexSell)) mean(accYl(indexSell)) mean(accZl(indexSell))]);
    %mean；数组的均值，前两秒加速度的均值
end

for i = 1:2000
    AHRSalgorithmr.UpdateIMU([0 0 0], [mean(accXr(indexSelr)) mean(accYr(indexSelr)) mean(accZr(indexSelr))]);
    %mean；数组的均值，前两秒加速度的均值
end

% For all data
for t = 1:length(timel)
    if(stationaryl(t))
        AHRSalgorithml.Kp = 0.4;%原本是0.5，0.8,0.4
    else
        AHRSalgorithml.Kp = 0;
    end
    AHRSalgorithml.UpdateIMU(([deg2rad(gyrXl(t)) deg2rad(gyrYl(t)) deg2rad(gyrZl(t))]), [accXl(t) accYl(t) accZl(t)]);%%deg2rad
    quatl(t,:) = AHRSalgorithml.Quaternion;%obj.Quaternion = obj.quaternConj(obj.q);
end


% For all data

for t = 1:length(time)
    if(stationaryr(t))
        AHRSalgorithmr.Kp = 0.25;%原本是0.5
    else
        AHRSalgorithmr.Kp = 0.1;
    end
    AHRSalgorithmr.Kp = 1;
%     AHRSalgorithmr.Updateyawl(([deg2rad(gyrXr(t)) deg2rad(gyrYr(t)) deg2rad(gyrZr(t))]), [mocap_euler(t,2) mocap_euler(t,3) mocap_euler(t,4)]);%%deg2rad
%     quatr(t,:) = AHRSalgorithmr.Quaternion;
    quatr(t,:) = quaternConj(euler2quatern(mocap_euler(t,2:4)));
end
% -------------------------------------------------------------------------
% Compute translational accelerations

% Rotate body accelerations to Earth frame
accl = quaternRotate([accXl accYl accZl], quaternConj(quatl));%obj.Quaternion = obj.quaternConj(obj.q);

% measurements to m/s/s
accl = accl * 9.81;

% Rotate body accelerations to Earth frame
accr = quaternRotate([accXr accYr accZr], quaternConj(quatr(start_plot:end_plot,:)));%obj.Quaternion = obj.quaternConj(obj.q);

% Convert acceleration measurements to m/s/s
accr = accr * 9.81;


% Compute translational velocities

accr(:,3) = accr(:,3) - 9.81;
accl(:,3) = accl(:,3) - 9.81;
vell = zeros(size(accl));
posl = zeros(size(accl));
% posl(1,:) = [-0.105,0.225,0.002];
posl(1,:) = [0,0,0];
velr = zeros(size(accr));
posr = zeros(size(accr));
% posr(1,:) = [0.105,-0.225,0.002];
posr(1,:) = [0,0,0];
[vell, posl,velr, posr] = dual_kalman_zupt(timel,vell, posl, stationaryl,accl,velr, posr, stationaryr,accr,samplePeriod,param(j));


posl=posl-posl(1,:);
posr=posr-posr(1,:);

% index_original = 1:size(groundtruth, 1);
% index_interp = linspace(1, size(groundtruth, 1), 6002);
% 
% % 对每一列进行插值
% groundtruth_interp = zeros(6002, size(groundtruth, 2));
% for col = 1:size(groundtruth, 2)
%     groundtruth_interp(:, col) = interp1(index_original, groundtruth(:, col), index_interp, 'spline');
% end
% 
% % 丢弃最后一行
% groundtruth_interp = groundtruth_interp(1:end-1, :);
% figure;
% plot3(groundtruth_interp(:,1), groundtruth_interp(:,2), groundtruth_interp(:,3), 'LineWidth', 2);
% rmsel=figerror(posl,groundtruth);
% rmser=figerror(posr,groundtruth);
%
%% 坐标系旋转
% pos0=[0.7512,-0.4727,0.2013
pos0=[0,0,0];
Rotation1 = rotz(-90,"deg");
Rotation2 = rotz(90,"deg");
% pos = (Rotation*pos')'+pos0;
% 修改对齐
plot_start=1;
plot_end=length(groundtruth);
% plot_end = 8100;
posl = (Rotation1*posl')'+pos0;
posr = (Rotation2*posr')'+pos0;
posl = posl(plot_start:plot_end,:);
posr = posr(plot_start:plot_end,:);
timel = timel(plot_start:plot_end);
timer = timer(plot_start:plot_end);
pos = (posr +posl)/2 ;

rmse(j,1)=param(j);
rmse(j,2)=figerror(pos(plot_start:plot_end,:),groundtruth(plot_start:plot_end,:));

% 修改
groundtruth = groundtruth(plot_start:plot_end,:);
end

% Plot translational accelerations
% figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Accelerationsl');
% hold on;
% plot(timel, accl(:,1), 'r');
% plot(timel, accl(:,2), 'g');
% plot(timel, accl(:,3), 'b');
% title('Accelerationl');
% xlabel('Time (s)');
% ylabel('Accelerationl (m/s/s)');
% legend('X', 'Y', 'Z');
% hold off;


% Plot translational accelerations
% figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Accelerationsl');
% hold on;
% plot(timer, accr(:,1), 'r');
% plot(timer, accr(:,2), 'g');
% plot(timer, accr(:,3), 'b');
% title('Accelerationl');
% xlabel('Time (s)');
% ylabel('Accelerationl (m/s/s)');
% legend('X', 'Y', 'Z');
% hold off;
% -------------------------------------------------------------------------
%% Plot translational velocity
% figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Velocity');
% hold on;
% plot(timel, vell(:,1), 'r');
% plot(timel, vell(:,2), 'g');
% plot(timel, vell(:,3), 'b');
% title('Velocityl');
% xlabel('Timel (s)');
% ylabel('Velocityl (m/s)');
% legend('X', 'Y', 'Z');
% hold off;
% 
% % Plot translational velocity
% figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Velocity');
% hold on;
% plot(timer, velr(:,1), 'r');
% plot(timer, velr(:,2), 'g');
% plot(timer, velr(:,3), 'b');
% title('Velocityr');
% xlabel('Time (s)');
% ylabel('Velocityr (m/s)');
% legend('X', 'Y', 'Z');
% hold off;


% 
% % -------------------------------------------------------------------------
% % Compute translational position
% 
% % Integrate velocity to yield position
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Position');
hold on;
plot(timel, posl(:,1), '.');
plot(timel, posl(:,2), '.');
plot(timel, posl(:,3), '.');
% plot(timer, pos(:,1), 'r');
% plot(timer, pos(:,2), 'g');
% plot(timer, pos(:,3), 'b');
plot(timer, posr(:,1), '--');
plot(timer, posr(:,2), '--');
plot(timer, posr(:,3),'--');
title('Position');
xlabel('Time (s)');
ylabel('Positionl (m)');
legend('Xl', 'Yl', 'Zl','Xr', 'Yr', 'Zr');
hold off;

figure
clf
plot3(pos(:,1), pos(:,2), pos(:,3), 'LineWidth', 2);
hold on;
% plot3(position(:,1), position(:,2), position(:,3), 'r-', 'LineWidth', 1);
% hold on;
plot3(posl(:,1), posl(:,2), posl(:,3), 'LineWidth', 2);
hold on ;
plot3(posr(:,1), posr(:,2), posr(:,3), 'LineWidth', 2);
hold on ;
plot3(groundtruth(:,1), groundtruth(:,2), groundtruth(:,3), 'LineWidth', 2,'DisplayName', 'Ground Truth');
hold on ;
plot(pos(1,1), pos(1,2),'rs')
title('Trajectory')
legend('dual-IMU','LF-IMU','RH-IMU','ground truth')
xlabel('X轴')
ylabel('Y轴')
zlabel('Z轴')
axis equal
hold off;
box on

        



 

 


