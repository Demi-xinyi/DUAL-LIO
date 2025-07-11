function [accL_world, accR_world] = estimate_attitude(accL, accR, gyrL, gyrR, timeL, timeR, stationaryL, stationaryR, config)
%ESTIMATE_ATTITUDE 

% Initialize AHRS structures
AHRS_L = AHRS('SamplePeriod', config.samplePeriod, 'Kp', 0.1, 'KpInit', 1);
AHRS_R = AHRS('SamplePeriod', config.samplePeriod, 'Kp', 0.01, 'KpInit', 1);

% Initial convergence 
idxL = find(timeL - timeL(1) < config.initPeriod);
idxR = find(timeR - timeR(1) < config.initPeriod);

if isempty(idxL) || isempty(idxR)
    error('Initial convergence duration too short or time vectors not valid.');
end

accL_mean = mean(accL(idxL, :), 1);
accR_mean = mean(accR(idxR, :), 1);

for i = 1:2000
    AHRS_L.UpdateIMU([0 0 0], accL_mean);
    AHRS_R.UpdateIMU([0 0 0], accR_mean);
end

% Allocate quaternion output
quatL = zeros(length(timeL), 4);
quatR = zeros(length(timeR), 4);

% Propagate orientation and compute quaternions
for t = 1:length(timeL)
    if stationaryL(t)
        AHRS_L.Kp = config.kpL;
    else
        AHRS_L.Kp = config.kpInitL;
    end
    gyroL = deg2rad(gyrL(t, :));
    AHRS_L.UpdateIMU(gyroL, accL(t, :));
    quatL(t, :) = AHRS_L.Quaternion;
end

for t = 1:length(timeR)
    if stationaryR(t)
        AHRS_R.Kp = config.kpR;
    else
        AHRS_R.Kp = config.kpInitR;
    end
    gyroR = deg2rad(gyrR(t, :));
    AHRS_R.UpdateIMU(gyroR, accR(t, :));
    quatR(t, :) = AHRS_R.Quaternion;
end

% Rotate body-frame acceleration to world-frame using quaternion
accL_world = quaternRotate(accL, quaternConj(quatL)) * 9.81;
accR_world = quaternRotate(accR, quaternConj(quatR)) * 9.81;

% Remove gravity from Z-axis
accL_world(:,3) = accL_world(:,3) - 9.81;
accR_world(:,3) = accR_world(:,3) - 9.81;
end