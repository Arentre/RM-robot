function result = identify_gimbal_yaw(inputData)
% Estimate the slow Yaw rigid-body model from an updated bench-test log.
% Model: Kt*i = J*d(omega)/dt + B*omega + constant disturbance torque.
if istable(inputData)
    data = inputData;
else
    data = readtable(inputData);
end
required = {'McuTime_s','YawRate_deg_s','YawActualCurrent_A','Phase'};
assert(all(ismember(required,data.Properties.VariableNames)), ...
    'Use a CSV produced by run_gimbal_bench_test with updated firmware.');

Kt = 0.741; % N m/A, GM6020 manual v1.4
t = data.McuTime_s;
w = deg2rad(data.YawRate_deg_s);
i = data.YawActualCurrent_A;
valid = isfinite(t) & isfinite(w) & isfinite(i);
t = t(valid); w = w(valid); i = i(valid);
phase = string(data.Phase(valid));
[t, uniqueIndex] = unique(t,'stable');
w = w(uniqueIndex); i = i(uniqueIndex); phase = phase(uniqueIndex);
assert(numel(t) >= 40 && all(diff(t) > 0), 'Not enough ordered MCU samples.');

dt = median(diff(t));
window = max(3,round(0.12/dt));
w = smoothdata(w,'movmean',max(3,round(0.06/dt)));
i = smoothdata(i,'movmean',max(3,round(0.04/dt)));
baselineRows = phase == "baseline" & abs(w) < deg2rad(2);
assert(nnz(baselineRows) >= 10, ...
    'Not enough stationary baseline samples to estimate current bias.');
biasCurrent = median(i(baselineRows));
driveTorque = Kt*(i-biasCurrent);
n = numel(t)-window;
A = zeros(n,2); y = zeros(n,1); label = strings(n,1);
for k = 1:n
    q = k:k+window;
    A(k,:) = [w(q(end))-w(q(1)),trapz(t(q),w(q))];
    y(k) = trapz(t(q),driveTorque(q));
    if all(phase(q) == phase(q(1)))
        label(k) = phase(q(1));
    else
        label(k) = "transition";
    end
end
excited = abs(A(:,1)) > deg2rad(0.4) | abs(y) > 2e-4;
estimateRows = excited & label == "positive_step";
validationRows = excited & label == "return_step";
if nnz(estimateRows) < 12 || nnz(validationRows) < 12
    idx = find(excited);
    cut = floor(0.6*numel(idx));
    estimateRows = false(n,1); validationRows = false(n,1);
    estimateRows(idx(1:cut)) = true;
    validationRows(idx(cut+1:end)) = true;
end
assert(nnz(estimateRows) >= 10 && nnz(validationRows) >= 5, ...
    'The test did not contain enough motion for identification.');

theta = A(estimateRows,:) \ y(estimateRows);
J = theta(1); B = theta(2); disturbance = Kt*biasCurrent;
if ~(isfinite(J) && J > 0 && isfinite(B) && B >= 0)
    error(['Identification produced nonphysical J or B. Repeat with a 1-2 deg ' ...
        'step and verify the actual-current channel.']);
end
prediction = A(validationRows,:)*theta;
truth = y(validationRows);
rmse = sqrt(mean((truth-prediction).^2));
relativeRmse = rmse/max(rms(truth),eps);
varianceEnergy = sum((truth-mean(truth)).^2);
if varianceEnergy > max(1e-12,1e-6*sum(truth.^2))
    r2 = 1-sum((truth-prediction).^2)/varianceEnergy;
else
    r2 = NaN; % R2 is undefined for an almost constant validation input.
end
b0 = Kt*3/J; % normalized command u corresponds nominally to +/-3 A

integralJ = J; integralB = B; model = [];
tfestEstimationFit = NaN; tfestValidationFit = NaN;
plantPole = NaN; plantBandwidth = NaN; plantDCGain = NaN;
method = "integral regression fallback";
try
    [toolFit,model] = fit_gimbal_rate_tfest(t,w,i,phase,Kt);
    J = toolFit.J_kg_m2;
    B = toolFit.B_Nm_s_rad;
    b0 = toolFit.B0_rad_s2_per_command;
    tfestEstimationFit = toolFit.EstimationFit_pct;
    tfestValidationFit = toolFit.ValidationFit_pct;
    plantPole = toolFit.PlantPole_rad_s;
    plantBandwidth = toolFit.PlantBandwidth_Hz;
    plantDCGain = toolFit.DCGain_rad_s_per_A;
    method = "System Identification Toolbox tfest";
catch ME
    warning('identify_gimbal_yaw:TFESTFallback', ...
        'tfest unavailable or invalid (%s); using integral regression.',ME.message);
end

result = struct('J_kg_m2',J,'B_Nm_s_rad',B, ...
    'BaselineCurrent_A',biasCurrent,'DisturbanceTorque_Nm',disturbance, ...
    'B0_rad_s2_per_command',b0, ...
    'TFESTEstimationFit_pct',tfestEstimationFit, ...
    'TFESTValidationFit_pct',tfestValidationFit, ...
    'PlantPole_rad_s',plantPole,'PlantBandwidth_Hz',plantBandwidth, ...
    'DCGain_rad_s_per_A',plantDCGain, ...
    'IntegralCrosscheckJ_kg_m2',integralJ, ...
    'IntegralCrosscheckB_Nm_s_rad',integralB, ...
    'ValidationR2',r2,'ValidationRelativeRmse',relativeRmse, ...
    'ValidationIntegralRmse_Nm_s',rmse, ...
    'SampleTime_s',dt,'Window_s',window*dt);
disp(struct2table(result,'AsArray',true));

here = fileparts(mfilename('fullpath'));
stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
save(fullfile(here,['gimbal_yaw_id_' stamp '.mat']), ...
    'result','data','model','method');
fprintf('Primary identification method: %s.\n',method);
fprintf(['Initial LADRC b0 estimate: %.1f rad/s^2 per normalized command. ' ...
    'Validate it on a separate run before changing firmware defaults.\n'],b0);
end
