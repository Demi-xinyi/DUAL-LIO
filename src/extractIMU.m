function [time, acc, gyr] = extractIMU(xIMUdata, startTime, stopTime)
%EXTRACTIMU 从xIMU对象提取裁剪后的加速度与角速度数据
%
% 输入：
%   xIMUdata : xIMUdataClass 
%   startTime, stopTime
%
% 输出：
%   time  
%   acc  
%   gyr  


time_all = xIMUdata.CalInertialAndMagneticData.Time;


acc_all = [xIMUdata.CalInertialAndMagneticData.Accelerometer.X, ...
           xIMUdata.CalInertialAndMagneticData.Accelerometer.Y, ...
           xIMUdata.CalInertialAndMagneticData.Accelerometer.Z];

gyr_all = [xIMUdata.CalInertialAndMagneticData.Gyroscope.X, ...
           xIMUdata.CalInertialAndMagneticData.Gyroscope.Y, ...
           xIMUdata.CalInertialAndMagneticData.Gyroscope.Z];


idx = find(sign(time_all-startTime)+1, 1) : find(sign(time_all-stopTime)+1, 1);


time = time_all(idx);
acc  = acc_all(idx, :);
gyr  = gyr_all(idx, :);
end
