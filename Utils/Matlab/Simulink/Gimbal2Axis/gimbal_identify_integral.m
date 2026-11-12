function result = gimbal_identify_integral(estimationInput,validationInput,phases,axis)
% 用最终 CAN 归一化指令拟合角速度增量，不依赖额外工具箱。
% 该增益是当前电机、驱动和负载下的经验量，不是电机转矩常数。
[A,y,stats] = windows(estimationInput,phases,axis);
[a,condition] = fit(A,y);
b = NaN(4,1); forward = NaN; reverse = NaN; drift = NaN;
reliable = false;
if ~isempty(validationInput)
    [V,z,~] = windows(validationInput,phases,axis);
    assert(~isequal(A,V) || ~isequal(y,z), ...
        'Use two separate acquisitions; validation repeats the estimation data.');
    [b,valCondition] = fit(V,z);
    forward = fitPercent(z,V*a);
    reverse = fitPercent(y,A*b);
    drift = abs(abs(a(1))-abs(b(1)))/mean(abs([a(1),b(1)]));
    reliable = forward >= 50 && reverse >= 50 && drift <= 0.30 && ...
        sign(a(1)) == sign(b(1)) && max(condition,valCondition) <= 100 && ...
        all(abs([a(1),b(1)]) >= 10 & abs([a(1),b(1)]) <= 2000);
end
recommended = NaN;
if reliable, recommended = mean(abs([a(1),b(1)])); end
result = struct('Reliable',reliable, ...
    'RecommendedB0_rad_s2_per_command',recommended, ...
    'B0_rad_s2_per_command',recommended, ...
    'EstimationB0',abs(a(1)),'ValidationB0',abs(b(1)), ...
    'CommandPolarity',sign(a(1)), ...
    'ViscousRate_s_inv',a(2),'CoulombAcceleration_rad_s2',a(3), ...
    'BiasAcceleration_rad_s2',a(4), ...
    'EstimationModelOnValidationFit_pct',forward, ...
    'ValidationModelOnEstimationFit_pct',reverse, ...
    'B0Difference_pct',100*drift,'Condition',condition, ...
    'EstimationWindows',size(A,1),'MedianSampleTime_s',stats(1), ...
    'MaxSampleGap_s',stats(2));
disp(struct2table(result,'AsArray',true));
if ~reliable
    fprintf('No b0 recommendation: use separate, sufficiently excited runs and check validation.\n');
end
end

function [A,y,stats] = windows(input,phases,axis)
if istable(input), d = input; else, d = readtable(input); end
names = d.Properties.VariableNames;
rate = [axis 'Rate_deg_s'];
output = [axis 'Output'];
assert(all(ismember({'McuTime_s',rate,output,'Phase'},names)), ...
    'Use raw %s bench or chirp telemetry, not gimbal_metrics.csv.',axis);
t = d.McuTime_s(:); w = deg2rad(d.(rate)); u = d.(output);
w = w(:); u = u(:);
phase = string(d.Phase);
valid = isfinite(t) & isfinite(w) & isfinite(u) & abs(u) <= 1 & ...
    ismember(phase,phases);
if ismember('StopReason',names), valid = valid & d.StopReason == 0; end
if ismember('Enable',names), valid = valid & d.Enable ~= 0; end
if ismember('PitchProtect',names), valid = valid & d.PitchProtect == 0; end
if strcmp(axis,'Pitch') && ismember('PitchLimitActive',names)
    valid = valid & d.PitchLimitActive == 0;
end
active = find(valid);
assert(numel(active) >= 30,'Too few valid moving-phase telemetry samples.');
dt = diff(t(active));
assert(all(dt >= 0) && any(dt > 0),'MCU timestamps went backwards; reacquire the run.');
step = median(dt(dt > 0));
stats = [step,max(dt)];
gap = min(0.10,3*step);
% 按 MCU 原始时间戳积分，不跨越大段丢帧或试验阶段切换。
% 用不重叠的时间窗，避免虚增样本数。
A = zeros(0,4); y = zeros(0,1);
window_s = 0.12;
if strcmp(axis,'Pitch'), window_s = 0.08; end
first = active(1);
for k = first+1:numel(t)
    if ~valid(k) || ~valid(k-1) || phase(k) ~= phase(k-1) || t(k)-t(k-1) > gap
        first = k;
        continue
    end
    if t(k)-t(first) < window_s, continue; end
    q = first:k;
    wq = w(q);
    % 速度接近零时平滑过渡摩擦符号，减轻陀螺仪量化的影响。
    friction = max(-1,min(1,wq/deg2rad(2)));
    if max(abs(wq)) >= deg2rad(3)
        A(end+1,:) = [trapz(t(q),u(q)), -trapz(t(q),wq), ...
            -trapz(t(q),friction), t(k)-t(first)]; %#ok<AGROW>
        y(end+1,1) = w(k)-w(first); %#ok<AGROW>
    end
    first = k;
end
assert(size(A,1) >= 16 && norm(y-mean(y)) > deg2rad(3), ...
    'Insufficient motion or telemetry continuity; use a longer sweep or more stepped runs.');
end

function [theta,condition] = fit(A,y)
scales = sqrt(sum(A.^2,1));
assert(all(scales > eps),'Identification has an unexcited input column.');
X = A./scales;
condition = cond(X);
assert(isfinite(condition) && condition <= 100, ...
    'Input, speed and friction are too correlated; use bidirectional excitation.');
weights = ones(size(y));
for iteration = 1:8
    % 枚举约束组合，要求阻尼和库仑摩擦非负。
    % 指令增益和恒定扰动允许带符号。
    bestLoss = Inf;
    for subset = 0:3
        cols = [1,find(bitget(subset,1:2))+1,4];
        rootWeight = sqrt(weights);
        c = (X(:,cols).*rootWeight) \ (y.*rootWeight);
        candidate = zeros(4,1); candidate(cols) = c;
        if any(candidate(2:3) < 0), continue; end
        e = y-X*candidate;
        loss = sum(weights.*e.^2);
        if loss < bestLoss
            bestLoss = loss; best = candidate;
        end
    end
    e = y-X*best;
    sigma = max(1.4826*median(abs(e-median(e))),1e-6);
    weights = min(1,(1.345*sigma)./max(abs(e),eps));
end
theta = best./scales(:);
end

function value = fitPercent(y,prediction)
value = 100*(1-norm(y-prediction)/max(norm(y-mean(y)),eps));
end
