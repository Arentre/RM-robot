function result = identify_gimbal_yaw_chirp(estimationInput,validationInput)
% Identify and independently validate the GM6020 Yaw rate plant.
% Each input must be a table or CSV from run_gimbal_chirp_test. One sweep
% should increase frequency and the other should decrease frequency.
assert(license('test','Identification_Toolbox') && exist('tfest','file') == 2, ...
    'System Identification Toolbox is required.');
estData = readInput(estimationInput);
valData = readInput(validationInput);
Kt = 0.741; % N m/A, GM6020 manual v1.4
[zEst,estInfo] = makeIdData(estData);
[zVal,valInfo] = makeIdData(valData);

opt = tfestOptions('Display','off');
opt.InitialCondition = 'estimate';
modelEst = tfest(zEst,1,0,opt);
modelVal = tfest(zVal,1,0,opt);
physicalEst = physicalParameters(modelEst,Kt);
physicalVal = physicalParameters(modelVal,Kt);
[yEstOnVal,fitEstOnVal] = compare(zVal,modelEst);
[~,fitValOnEst] = compare(zEst,modelVal);
[~,fitEstSelf] = compare(zEst,modelEst);
[~,fitValSelf] = compare(zVal,modelVal);

jDifference_pct = relativeDifference(physicalEst.J_kg_m2,physicalVal.J_kg_m2);
b0Difference_pct = relativeDifference( ...
    physicalEst.B0_rad_s2_per_command,physicalVal.B0_rad_s2_per_command);
[whiteFraction,inputIndependentFraction,maxInputCorrelation,confidence] = ...
    residualDiagnostics(zVal,yEstOnVal);
recommendedB0 = median([physicalEst.B0_rad_s2_per_command, ...
    physicalVal.B0_rad_s2_per_command]);
reliable = fitEstOnVal(1) >= 50 && fitValOnEst(1) >= 50 && ...
    jDifference_pct <= 30 && b0Difference_pct <= 30 && ...
    inputIndependentFraction >= 0.80;

result = struct( ...
    'Reliable',logical(reliable), ...
    'RecommendedB0_rad_s2_per_command',recommendedB0, ...
    'EstimationJ_kg_m2',physicalEst.J_kg_m2, ...
    'ValidationJ_kg_m2',physicalVal.J_kg_m2, ...
    'EstimationB_Nm_s_rad',physicalEst.B_Nm_s_rad, ...
    'ValidationB_Nm_s_rad',physicalVal.B_Nm_s_rad, ...
    'EstimationB0',physicalEst.B0_rad_s2_per_command, ...
    'ValidationB0',physicalVal.B0_rad_s2_per_command, ...
    'EstimationSelfFit_pct',fitEstSelf(1), ...
    'ValidationSelfFit_pct',fitValSelf(1), ...
    'EstimationModelOnValidationFit_pct',fitEstOnVal(1), ...
    'ValidationModelOnEstimationFit_pct',fitValOnEst(1), ...
    'JDifference_pct',jDifference_pct, ...
    'B0Difference_pct',b0Difference_pct, ...
    'ResidualWhiteFraction',whiteFraction, ...
    'ResidualInputIndependentFraction',inputIndependentFraction, ...
    'MaxResidualInputCorrelation',maxInputCorrelation, ...
    'Correlation95Limit',confidence, ...
    'EstimationSampleTime_s',estInfo.SampleTime_s, ...
    'ValidationSampleTime_s',valInfo.SampleTime_s, ...
    'EstimationCurrentBias_A',estInfo.CurrentBias_A, ...
    'ValidationCurrentBias_A',valInfo.CurrentBias_A);
disp(struct2table(result,'AsArray',true));

t = (0:size(zVal.OutputData,1)-1)'*zVal.Ts;
f = figure('Color','w','Name','Yaw chirp identification validation', ...
    'Position',[100 80 1200 780]);
tiledlayout(3,1,'TileSpacing','compact');
nexttile; plot(t,rad2deg(zVal.OutputData),'b', ...
    t,rad2deg(yEstOnVal.OutputData),'r--');
grid on; ylabel('deg/s'); legend('Measured','Independent prediction');
title(sprintf('Validation fit %.1f%%',fitEstOnVal(1)));
nexttile; plot(t,zVal.InputData,'k'); grid on; ylabel('A');
title('Measured torque current input');
nexttile; plot(t,rad2deg(zVal.OutputData-yEstOnVal.OutputData),'b');
grid on; ylabel('deg/s'); xlabel('Time (s)'); title('Validation residual');

here = fileparts(mfilename('fullpath'));
stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
pngPath = fullfile(here,['gimbal_chirp_id_' stamp '.png']);
matPath = fullfile(here,['gimbal_chirp_id_' stamp '.mat']);
exportgraphics(f,pngPath,'Resolution',160);
save(matPath,'result','modelEst','modelVal','zEst','zVal');
if reliable
    fprintf('Identification accepted. Recommended LADRC b0: %.1f.\n', ...
        recommendedB0);
else
    fprintf(['Identification rejected by reliability gates. Keep the current ' ...
        'firmware b0 and inspect the validation plot.\n']);
end
end

function data = readInput(input)
if istable(input)
    data = input;
else
    data = readtable(input);
end
required = {'McuTime_s','YawRate_deg_s','YawActualCurrent_A','Phase'};
assert(all(ismember(required,data.Properties.VariableNames)), ...
    'Input must come from run_gimbal_chirp_test.');
end

function [z,info] = makeIdData(data)
phase = string(data.Phase);
baseline = phase == "baseline" & abs(data.YawRate_deg_s) < 1;
active = phase == "chirp";
assert(nnz(baseline) >= 50 && nnz(active) >= 500, ...
    'The log does not contain enough baseline or chirp data.');
bias = median(data.YawActualCurrent_A(baseline));
t = data.McuTime_s(active);
y = deg2rad(data.YawRate_deg_s(active));
u = data.YawActualCurrent_A(active)-bias;
valid = isfinite(t) & isfinite(y) & isfinite(u);
t = t(valid); y = y(valid); u = u(valid);
[t,index] = unique(t,'stable'); y = y(index); u = u(index);
Ts = 0.005; % 200 Hz is ample for the 2 Hz excitation and reduces USB jitter.
tu = (t(1):Ts:t(end)).';
yu = interp1(t,y,tu,'pchip');
uu = interp1(t,u,tu,'linear');
z = iddata(yu,uu,Ts,'TimeUnit','s', ...
    'InputName','measured torque current','InputUnit','A', ...
    'OutputName','Yaw angular rate','OutputUnit','rad/s');
z = detrend(z,0);
info = struct('SampleTime_s',Ts,'CurrentBias_A',bias);
end

function p = physicalParameters(model,Kt)
plantPole = pole(model);
gain = dcgain(model);
assert(isscalar(plantPole) && isreal(plantPole) && plantPole < 0 && ...
    isfinite(gain) && gain > 0,'A fitted model is not physically valid.');
J = -Kt/(gain*plantPole);
B = Kt/gain;
p = struct('J_kg_m2',double(J),'B_Nm_s_rad',double(B), ...
    'B0_rad_s2_per_command',double(3*Kt/J));
end

function value = relativeDifference(a,b)
value = 100*abs(a-b)/max((abs(a)+abs(b))/2,eps);
end

function [whiteFraction,inputFraction,maxInputCorrelation,limit] = ...
        residualDiagnostics(data,prediction)
e = data.OutputData-prediction.OutputData;
u = data.InputData;
e = e-mean(e); u = u-mean(u);
maxLag = 25;
auto = zeros(maxLag,1); cross = zeros(maxLag+1,1);
for lag = 1:maxLag
    auto(lag) = corr(e(1:end-lag),e(1+lag:end));
end
for lag = 0:maxLag
    cross(lag+1) = corr(e(1+lag:end),u(1:end-lag));
end
limit = 1.96/sqrt(numel(e));
whiteFraction = mean(abs(auto) <= limit);
inputFraction = mean(abs(cross) <= limit);
maxInputCorrelation = max(abs(cross));
end
