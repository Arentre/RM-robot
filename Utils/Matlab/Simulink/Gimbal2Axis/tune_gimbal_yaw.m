function summary = tune_gimbal_yaw(port, b0, wcCandidates)
% Conservative runtime LADRC sweep. Firmware remains disabled after each run.
if nargin < 3, wcCandidates = [2 4 6 8]; end
validateattributes(b0, {'numeric'}, {'scalar','finite','>=',10,'<=',2000});
validateattributes(wcCandidates, {'numeric'}, ...
    {'vector','finite','positive','<=',20});
s = getPort(port);
configureTerminator(s,'LF');
summary = table();
for k = 1:numel(wcCandidates)
    wc = wcCandidates(k);
    r = min(max(wc/2,2),6);
    w0 = 4*wc;
    limit = 0.10; % 0.30 A nominal command ceiling
    writeline(s,sprintf('YPARAM=%.6g,%.6g,%.6g,%.6g,%.6g', ...
        r,b0,wc,w0,limit));
    pause(0.15);
    trial = run_gimbal_bench_test(s,30.0);
    positive = trial.Phase == 'positive_step';
    returning = trial.Phase == 'return_step';
    endPositive = lastWindow(trial,positive,0.5);
    endReturn = lastWindow(trial,returning,0.5);
    positiveStart = trial.YawFeedback_deg(find(positive,1));
    direction = sign(mean(trial.YawTarget_deg(positive))-positiveStart);
    normalized = direction*(trial.YawFeedback_deg(positive)-positiveStart);
    overshoot = max(0,(max(normalized)-30.0)/30.0*100);
    row = table(wc,r,w0,limit,mean(abs(endPositive.YawError_deg)), ...
        mean(abs(endReturn.YawError_deg)),overshoot, ...
        max(abs(trial.YawRate_deg_s)),max(abs(trial.YawActualCurrent_A)), ...
        'VariableNames',{'wc','r','w0','OutputLimit','PositiveFinalError_deg', ...
        'ReturnFinalError_deg','Overshoot_pct','PeakRate_deg_s','PeakCurrent_A'});
    summary = [summary;row]; %#ok<AGROW>
end
summary.Score = summary.PositiveFinalError_deg + summary.ReturnFinalError_deg + ...
    0.02*summary.Overshoot_pct + 0.002*summary.PeakRate_deg_s + ...
    0.2*summary.PeakCurrent_A;
[~,best] = min(summary.Score);
bestRow = summary(best,:);
writeline(s,sprintf('YPARAM=%.6g,%.6g,%.6g,%.6g,%.6g', ...
    bestRow.r,b0,bestRow.wc,bestRow.w0,bestRow.OutputLimit));
writeline(s,'STOP');
disp(summary);
fprintf('Selected runtime parameters: r=%.3g, b0=%.3g, wc=%.3g, w0=%.3g, limit=%.3g\n', ...
    bestRow.r,b0,bestRow.wc,bestRow.w0,bestRow.OutputLimit);
here = fileparts(mfilename('fullpath'));
stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
writetable(summary,fullfile(here,['gimbal_yaw_tuning_' stamp '.csv']));
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
