function result = identify_gimbal_pitch(inputData)
% Estimate Pitch inertia/friction plus a constant gravity torque per run.
if istable(inputData), data = inputData; else, data = readtable(inputData); end
required = {'McuTime_s','PitchRate_deg_s','PitchActualCurrent_A','Phase'};
assert(all(ismember(required,data.Properties.VariableNames)), ...
    'Use a CSV produced by run_gimbal_pitch_bench_test.');

Kt = 0.741;
t = data.McuTime_s; w = deg2rad(data.PitchRate_deg_s);
i = data.PitchActualCurrent_A; phase = string(data.Phase);
valid = isfinite(t) & isfinite(w) & isfinite(i);
t = t(valid); w = w(valid); i = i(valid); phase = phase(valid);
[t,idx] = unique(t,'stable'); w = w(idx); i = i(idx); phase = phase(idx);
assert(numel(t) >= 40 && all(diff(t) > 0),'Not enough ordered MCU samples.');
dt = median(diff(t)); window = max(3,round(0.12/dt));
w = smoothdata(w,'movmean',max(3,round(0.06/dt)));
i = smoothdata(i,'movmean',max(3,round(0.04/dt)));
baselineRows = phase == "baseline" & abs(w) < deg2rad(2);
assert(nnz(baselineRows) >= 10, ...
    'Not enough stationary baseline samples to estimate gravity current.');
biasCurrent = median(i(baselineRows));
driveTorque = Kt*(i-biasCurrent);
n = numel(t)-window; A = zeros(n,2); y = zeros(n,1); label = strings(n,1);
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
excited = abs(A(:,1)) > deg2rad(0.25) | abs(y) > 1e-4;
estimateRows = excited & label == "positive_step";
validationRows = excited & label == "return_step";
assert(nnz(estimateRows) >= 10 && nnz(validationRows) >= 5, ...
    'Not enough Pitch motion. Keep supporting the payload and repeat the test.');
theta = A(estimateRows,:)\y(estimateRows);
J = theta(1); B = theta(2); gravity = Kt*biasCurrent;
if ~(isfinite(J) && J > 0 && isfinite(B) && B >= 0)
    error('Pitch identification is nonphysical; verify current scale and motion.');
end
truth = y(validationRows); prediction = A(validationRows,:)*theta;
rmse = sqrt(mean((truth-prediction).^2));
relativeRmse = rmse/max(rms(truth),eps);
varianceEnergy = sum((truth-mean(truth)).^2);
if varianceEnergy > max(1e-12,1e-6*sum(truth.^2))
    r2 = 1-sum((truth-prediction).^2)/varianceEnergy;
else
    r2 = NaN;
end
b0 = Kt*3/J;
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
    warning('identify_gimbal_pitch:TFESTFallback', ...
        'tfest unavailable or invalid (%s); using integral regression.',ME.message);
end
result = struct('J_kg_m2',J,'B_Nm_s_rad',B, ...
    'BaselineCurrent_A',biasCurrent,'GravityAndBiasTorque_Nm',gravity, ...
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
save(fullfile(here,['gimbal_pitch_id_' stamp '.mat']), ...
    'result','data','model','method');
fprintf('Primary identification method: %s.\n',method);
end
