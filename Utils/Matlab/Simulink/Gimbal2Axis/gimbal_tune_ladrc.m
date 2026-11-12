function summary = gimbal_tune_ladrc(port,axis,b0,wcCandidates,step_deg,outputLimit)
% 双轴共用的监督扫参流程，失败时恢复原参数。
if strcmp(axis,'Y')
    collect = @run_gimbal_bench_test; name = 'Yaw'; maxStep = 10; maxWc = 20;
else
    collect = @run_gimbal_pitch_bench_test; name = 'Pitch'; maxStep = 5; maxWc = 10;
end
validateattributes(outputLimit,{'numeric'}, ...
    {'scalar','real','finite','>=',0.005,'<=',1});
validateattributes(step_deg,{'numeric'}, ...
    {'scalar','real','finite','>=',2,'<=',maxStep});
validateattributes(b0, {'numeric'}, {'scalar','finite','>=',10,'<=',2000});
validateattributes(wcCandidates, {'numeric'}, ...
    {'vector','nonempty','finite','positive','<=',maxWc});
s = getPort(port);
configureTerminator(s,'LF');
s.Timeout = 0.25;
gimbal_usb_command(s,'STOP','ack:STOP');
original = gimbal_read_axis_params(s,axis);
restoreGuard = onCleanup(@() restoreAxis(s,axis,original));
summary = table();
for k = 1:numel(wcCandidates)
    wc = wcCandidates(k);
    r = min(max(wc/2,2),6);
    w0 = 4*wc;
    limit = outputLimit;
    setAxis(s,axis,[r,b0,wc,w0,limit]);
    trial = collect(s,step_deg,limit);
    positive = trial.Phase == 'positive_step';
    returning = trial.Phase == 'return_step';
    endPositive = lastWindow(trial,positive,0.5);
    endReturn = lastWindow(trial,returning,0.5);
    feedback = trial.([name 'Feedback_deg']);
    errorField = [name 'Error_deg'];
    base = trial.Phase == 'baseline';
    origin = median(feedback(base));
    response = mod(feedback(positive)-origin+180,360)-180;
    overshoot = max(0,(max(response)-step_deg)/step_deg*100);
    reached = max(response) >= step_deg-0.75 && min(response) > -0.5;
    row = table(wc,r,w0,limit,mean(abs(endPositive.(errorField))), ...
        mean(abs(endReturn.(errorField))),overshoot, ...
        max(abs(trial.([name 'Rate_deg_s']))),max(abs(trial.([name 'ActualCurrent_A']))),reached, ...
        'VariableNames',{'wc','r','w0','OutputLimit','PositiveFinalError_deg', ...
        'ReturnFinalError_deg','Overshoot_pct','PeakRate_deg_s','PeakCurrent_A','Reached'});
    summary = [summary;row]; %#ok<AGROW>
end
summary.Score = summary.PositiveFinalError_deg + summary.ReturnFinalError_deg + ...
    0.02*summary.Overshoot_pct + 0.002*summary.PeakRate_deg_s + ...
    0.2*summary.PeakCurrent_A;
summary.Score(~summary.Reached | summary.PositiveFinalError_deg > 0.75 | ...
    summary.ReturnFinalError_deg > 0.75 | summary.Overshoot_pct > 35) = Inf;
[score,best] = min(summary.Score);
summary.Selected = false(height(summary),1);
clear restoreGuard;
if isfinite(score)
    bestRow = summary(best,:);
    try
        setAxis(s,axis,[bestRow.r,b0,bestRow.wc,bestRow.w0,bestRow.OutputLimit]);
        summary.Selected(best) = true;
    catch ME
        restoreAxis(s,axis,original);
        rethrow(ME);
    end
else
    fprintf('No %s candidate reached and settled at both targets; original settings restored.\n',name);
end
disp(summary);
here = fileparts(mfilename('fullpath'));
stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
writetable(summary,fullfile(here,['gimbal_' lower(name) '_tuning_' stamp '.csv']));
end

function setAxis(s,axis,p)
gimbal_usb_command(s,'STOP','ack:STOP');
command = [axis 'PARAM'];
gimbal_usb_command(s,sprintf('%s=%.6g,%.6g,%.6g,%.6g,%.6g',command,p),['ack:' command]);
end

function restoreAxis(s,axis,p)
try
    setAxis(s,axis,p);
catch ME
    warning('gimbal_tune_ladrc:RestoreFailed', ...
        'Could not confirm restored parameters: %s',ME.message);
end
end

function out = lastWindow(data,mask,seconds)
part = data(mask,:);
tEnd = part.McuTime_s(end);
out = part(part.McuTime_s >= tEnd-seconds,:);
end

function s = getPort(port)
if ischar(port) || (isstring(port) && isscalar(port))
    s = serialport(port,115200);
elseif isobject(port) && isprop(port,'NumBytesAvailable')
    s = port;
else
    error('port must be a COM port name or an existing serialport object.');
end
end
