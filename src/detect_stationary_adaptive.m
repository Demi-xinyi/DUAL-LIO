function [stationaryL, stationaryR] = detect_stationary_adaptive(accL, accR, gyrL, gyrR, config)
%DETECT_STATIONARY_ADAPTIVE 
%
% 输入：
%   accL, accR : 加速度
%   gyrL, gyrR : 角速度
%   config     : 包含 samplePeriod, windowLen_sec, slipstride, minPeakHeight
%
% 输出：
%   stationaryL, stationaryR : 逻辑向量，表示每时刻是否为静止


sampleRate = 1 / config.samplePeriod;
windowLen = round(config.windowLen_sec * sampleRate);
slipstride = config.slipstride;
minPeakHeight = config.minPeakHeight;

[b_lp, a_lp]   = butter(1, (2*5)/sampleRate, 'low');
[b_hp, a_hp]   = butter(1, (2*10)/sampleRate, 'high');
[b_lp2, a_lp2] = butter(1, (2*11)/sampleRate, 'low');

accMagL = sqrt(sum(accL.^2, 2));
accMagR = sqrt(sum(accR.^2, 2));
gyrMagL = sqrt(sum(gyrL.^2, 2));
gyrMagR = sqrt(sum(gyrR.^2, 2));

accMagL_filt = filtfilt(b_lp, a_lp, accMagL);
accMagR_filt = filtfilt(b_lp, a_lp, accMagR);
gyrL_filt = abs(filtfilt(b_hp, a_hp, gyrMagL));
gyrR_filt = abs(filtfilt(b_hp, a_hp, gyrMagR));
gyrL_filt = filtfilt(b_lp2, a_lp2, gyrL_filt);
gyrR_filt = filtfilt(b_lp2, a_lp2, gyrR_filt);
gyrDiffL = [0; abs(diff(gyrL_filt))];
gyrDiffR = [0; abs(diff(gyrR_filt))];

N = length(accMagL);
stationaryL = false(N,1);
stationaryR = false(N,1);

for i = 1:slipstride:(N - windowLen)
    idx = i:(i + windowLen - 1);

    % -- LF --
    segmentL = accMagL_filt(idx);
    [~, locsL] = findpeaks(segmentL, ...
        'MinPeakHeight', minPeakHeight, ...
        'MinPeakDistance', round(0.3 * sampleRate));
    localFreqL = length(locsL) / config.windowLen_sec;
    threshL = 0.03 + 0.008 * (localFreqL - 1);  % 自适应角速度阈值

    statL = (gyrL_filt(idx) < threshL) & (gyrDiffL(idx) < 0.02);
    stationaryL(idx) = stationaryL(idx) | statL;

    % -- RH --
    segmentR = accMagR_filt(idx);
    [~, locsR] = findpeaks(segmentR, ...
        'MinPeakHeight', minPeakHeight, ...
        'MinPeakDistance', round(0.3 * sampleRate));
    localFreqR = length(locsR) / config.windowLen_sec;
    threshR = 0.03 + 0.008 * (localFreqR - 1);

    statR = (gyrR_filt(idx) < threshR) & (gyrDiffR(idx) < 0.02);
    stationaryR(idx) = stationaryR(idx) | statR;
end
end
