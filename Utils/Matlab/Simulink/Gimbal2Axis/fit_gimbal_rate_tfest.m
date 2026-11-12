function [fit,model] = fit_gimbal_rate_tfest(t,w,current,phase,Kt)
% 使用系统辨识工具箱拟合电流到角速度模型 Kt/(J*s+B)。
assert(license('test','Identification_Toolbox') && exist('tfest','file') == 2, ...
    'System Identification Toolbox is unavailable.');
phase = string(phase);
baseline = phase == "baseline" & abs(w) < deg2rad(2);
assert(nnz(baseline) >= 10,'Not enough stationary baseline samples.');
biasCurrent = median(current(baseline));
u = current-biasCurrent;
Ts = median(diff(t));
zEst = phaseData(t,w,u,phase == "positive_step",Ts);
zVal = phaseData(t,w,u,phase == "return_step",Ts);

opt = tfestOptions('Display','off');
opt.InitialCondition = 'estimate';
model = tfest(zEst,1,0,opt);
p = pole(model);
dc = dcgain(model);
assert(isscalar(p) && isreal(p) && p < 0 && isfinite(dc) && dc > 0, ...
    'tfest produced a nonphysical first-order motor/load model.');
J = -Kt/(dc*p);
B = Kt/dc;
[~,estFit] = compare(zEst,model);
[~,valFit] = compare(zVal,model);
fit = struct('J_kg_m2',double(J),'B_Nm_s_rad',double(B), ...
    'B0_rad_s2_per_command',double(Kt*3/J), ...
    'PlantPole_rad_s',double(p),'PlantBandwidth_Hz',double(-p/(2*pi)), ...
    'DCGain_rad_s_per_A',double(dc), ...
    'BaselineCurrent_A',biasCurrent,'EstimationFit_pct',double(estFit(1)), ...
    'ValidationFit_pct',double(valFit(1)),'SampleTime_s',Ts);
end

function z = phaseData(t,y,u,mask,Ts)
phaseIndex = find(mask);
phaseRate = y(phaseIndex);
moving = find(abs(phaseRate) > deg2rad(1));
assert(~isempty(moving),'No useful motion was found in a test phase.');
margin = max(2,round(0.10/Ts));
first = max(1,moving(1)-margin);
last = min(numel(phaseIndex),moving(end)+margin);
activeIndex = phaseIndex(first:last);
tp = t(activeIndex); yp = y(activeIndex); up = u(activeIndex);
assert(numel(tp) >= 20,'A test phase has too few samples for tfest.');
tu = (tp(1):Ts:tp(end)).';
yu = interp1(tp,yp,tu,'pchip');
uu = interp1(tp,up,tu,'previous');
z = iddata(yu,uu,Ts,'TimeUnit','s', ...
    'InputName','torque current','InputUnit','A', ...
    'OutputName','angular rate','OutputUnit','rad/s');
end
