function pos = run_lio(config, timeL, accL, gyrL, timeR, accR, gyrR)
%RUN_MAIN 
%
% 输入：
%   config : 参数配置结构体
%   timeL, accL, gyrL 
%   timeR, accR, gyrR 
%
% 输出：
%   pos 

[stationaryL, stationaryR] = detect_stationary_adaptive(accL, accR, gyrL, gyrR, config);

accL = accL / 9.81;
accR = accR / 9.81;

gyrL = rad2deg(gyrL);
gyrR = rad2deg(gyrR);


[accLw, accRw] = estimate_attitude(accL, accR, gyrL, gyrR, timeL, timeR, stationaryL, stationaryR, config);

vell = zeros(size(accLw));
posl = zeros(size(accLw));
posl(1,:) = [0,0,0];
velr = zeros(size(accRw));
posr = zeros(size(accRw));
posr(1,:) = [0,0,0];
pos = estimate_trajectory(timeL,vell, posl, stationaryL,accLw,velr, posr, stationaryR,accRw,config);

end
