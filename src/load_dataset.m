function [timeL, accL, gyrL, timeR, accR, gyrR, groundtruth] = load_dataset(filePathL, filePathR, truthPath, config)
%LOAD_DATASET 
%
% 输入：
%   filePathL, filePathR 
%   truthPath            
%   config               
%
% 输出：
%   timeL, accL, gyrL    
%   timeR, accR, gyrR    
%   groundtruth          

% --- 加载左右IMU数据
xIMUdataL = xIMUdataClass(filePathL, 'InertialMagneticSampleRate', 1/config.samplePeriod);
xIMUdataR = xIMUdataClass(filePathR, 'InertialMagneticSampleRate', 1/config.samplePeriod);

[timeL, accL, gyrL] = extractIMU(xIMUdataL, config.startTime, config.stopTime);
[timeR, accR, gyrR] = extractIMU(xIMUdataR, config.startTime, config.stopTime);

clear('xIMUdataL');
clear('xIMUdataR');


% --- 加载 Ground Truth 数据
data = readmatrix(truthPath);
groundtruth = data(1:length(accL), 1:3);
end
