function summary = tune_gimbal_friction(port,staticCandidates,coulombRatios)
% 将正反向 Yaw 响应与无补偿的 2° 基线进行对比。
if nargin < 2, staticCandidates = [0.040 0.060 0.080]; end
if nargin < 3, coulombRatios = [0.35 0.60]; end
validateattributes(staticCandidates,{'numeric'}, ...
    {'vector','nonempty','real','finite','positive','<=',0.10});
validateattributes(coulombRatios,{'numeric'}, ...
    {'vector','nonempty','real','finite','>=',0,'<=',1});
if ischar(port) || (isstring(port) && isscalar(port))
    s = serialport(port,115200);
else
    s = port;
end
configureTerminator(s,'LF');
s.Timeout = 0.25;
gimbal_usb_command(s,'STOP','ack:STOP');

state = gimbal_usb_command(s,'GET','ack:GET,');
fields = regexp(state,',yff=([^,]+,[^,]+,[^,]+,[^,]+,[^,]+,[^,]+),', ...
    'tokens','once');
assert(~isempty(fields),'GET did not include Yaw friction settings.');
original = sscanf(fields{1},'%f,%f,%f,%f,%f,%f').';
assert(numel(original) == 6 && all(isfinite(original)), ...
    'Invalid original Yaw friction settings.');
restoreGuard = onCleanup(@() restoreFriction(s,original));
[S,R] = ndgrid(staticCandidates,coulombRatios);
candidates = unique([0 0; S(:) S(:).*R(:)],'rows','stable');
summary = table();
for k = 1:size(candidates,1)
    pair = candidates(k,:);
    setFriction(s,k ~= 1,[pair original(4:6)]);
    [positive,negative] = runPair(s);
    score = max(positive,negative); % 按较差方向评分。
    row = table(pair(1),pair(2),positive,negative,score, ...
        'VariableNames',{'StaticFF','CoulombFF','PositiveScore','NegativeScore','Score'});
    summary = [summary;row]; %#ok<AGROW>
end
summary.Selected = false(height(summary),1);
summary.ConfirmationScore = NaN(height(summary),1);
[score,best] = min(summary.Score(2:end));
best = best+1;
if isfinite(score) && score < 0.90*summary.Score(1)
    setFriction(s,true,[candidates(best,:) original(4:6)]);
    [positive,negative] = runPair(s);
    confirmation = max(positive,negative);
    summary.ConfirmationScore(best) = confirmation;
    if isfinite(confirmation) && confirmation < 0.90*summary.Score(1)
        summary.Selected(best) = true;
        fprintf('Verified runtime Yaw friction: static %.4f, Coulomb %.4f.\n', ...
            candidates(best,1),candidates(best,2));
    end
end
% 先恢复原参数，只有通过复测的候选才会保留。
clear restoreGuard;
if any(summary.Selected)
    try
        setFriction(s,true,[candidates(best,:) original(4:6)]);
    catch ME
        restoreFriction(s,original);
        rethrow(ME);
    end
else
    fprintf('No verified improvement over baseline; restored original settings.\n');
end
disp(summary);
here = fileparts(mfilename('fullpath'));
stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
writetable(summary,fullfile(here,['gimbal_friction_tuning_' stamp '.csv']));

end

function restoreFriction(s,original)
try
    setFriction(s,original(1),original(2:6));
catch ME
    warning('tune_gimbal_friction:RestoreFailed', ...
        'Could not confirm restored friction settings: %s',ME.message);
end
end

function setFriction(s,enable,p)
gimbal_usb_command(s,'STOP','ack:STOP');
gimbal_usb_command(s,sprintf('YFFPARAM=%.6g,%.6g,%.6g,%.6g,%.6g',p), ...
    'ack:YFFPARAM');
gimbal_usb_command(s,sprintf('YFF=%d',enable),'ack:YFF=');
end

function [positive,negative] = runPair(s)
positive = trialScore(run_gimbal_bench_test(s,2),2);
negative = trialScore(run_gimbal_bench_test(s,-2),-2);
end

function score = trialScore(d,step)
score = Inf;
phase = string(d.Phase);
base = d(phase == "baseline",:);
p = d(phase == "positive_step",:);
r = d(phase == "return_step",:);
if isempty(base) || isempty(p) || isempty(r), return; end
% 用 MCU 目标实际变化时刻计时，而不是主机写串口的时刻。
origin = median(base.YawFeedback_deg(base.McuTime_s >= base.McuTime_s(end)-0.3));
wrap = @(x) mod(x+180,360)-180;
start = find(sign(step)*wrap(p.YawTarget_deg-origin) >= 0.75*abs(step),1);
if isempty(start), return; end
p = p(start:end,:);
response = sign(step)*wrap(p.YawFeedback_deg-origin);
moving = find(response >= 0.20*abs(step),1);
if isempty(moving) || max(response) < abs(step)-0.75 || min(response) < -0.4
    return;
end
tail = p.McuTime_s >= p.McuTime_s(end)-0.5;
back = r.McuTime_s >= r.McuTime_s(end)-0.5;
finalError = mean(abs(p.YawError_deg(tail)));
returnError = mean(abs(r.YawError_deg(back)));
if max(finalError,returnError) > 0.75, return; end
delay = p.McuTime_s(moving)-p.McuTime_s(1);
overshoot = max(0,max(response)-abs(step));
score = delay+0.6*overshoot+finalError+returnError + ...
    0.15*max(abs(d.YawActualCurrent_A));
end
