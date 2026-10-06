function summary = tune_gimbal_pitch(port,b0,wcCandidates)
% Conservative Pitch LADRC sweep. Keep supporting the payload throughout.
if nargin < 3, wcCandidates = [1 2 3 4]; end
validateattributes(b0,{'numeric'},{'scalar','finite','>=',10,'<=',2000});
validateattributes(wcCandidates,{'numeric'},{'vector','finite','positive','<=',10});
s = getPort(port); configureTerminator(s,'LF'); summary = table();
for k = 1:numel(wcCandidates)
    wc = wcCandidates(k); r = min(max(wc/2,1),4); w0 = 4*wc; limit = 0.08;
    writeline(s,sprintf('PPARAM=%.6g,%.6g,%.6g,%.6g,%.6g', ...
        r,b0,wc,w0,limit));
    pause(0.15);
    trial = run_gimbal_pitch_bench_test(s,0.5);
    positive = trial.Phase == 'positive_step'; returning = trial.Phase == 'return_step';
    endPositive = lastWindow(trial,positive,0.5); endReturn = lastWindow(trial,returning,0.5);
    start = trial.PitchFeedback_deg(find(positive,1));
    direction = sign(mean(trial.PitchTarget_deg(positive))-start);
    normalized = direction*(trial.PitchFeedback_deg(positive)-start);
    overshoot = max(0,(max(normalized)-0.5)/0.5*100);
    row = table(wc,r,w0,limit,mean(abs(endPositive.PitchError_deg)), ...
        mean(abs(endReturn.PitchError_deg)),overshoot, ...
        max(abs(trial.PitchRate_deg_s)),max(abs(trial.PitchActualCurrent_A)), ...
        'VariableNames',{'wc','r','w0','OutputLimit','PositiveFinalError_deg', ...
        'ReturnFinalError_deg','Overshoot_pct','PeakRate_deg_s','PeakCurrent_A'});
    summary = [summary;row]; %#ok<AGROW>
end
summary.Score = summary.PositiveFinalError_deg+summary.ReturnFinalError_deg+ ...
    0.02*summary.Overshoot_pct+0.003*summary.PeakRate_deg_s+ ...
    0.2*summary.PeakCurrent_A;
[~,best] = min(summary.Score); bestRow = summary(best,:);
writeline(s,sprintf('PPARAM=%.6g,%.6g,%.6g,%.6g,%.6g', ...
    bestRow.r,b0,bestRow.wc,bestRow.w0,bestRow.OutputLimit));
writeline(s,'STOP'); disp(summary);
fprintf('Selected Pitch runtime parameters: r=%.3g, b0=%.3g, wc=%.3g, w0=%.3g, limit=%.3g\n', ...
    bestRow.r,b0,bestRow.wc,bestRow.w0,bestRow.OutputLimit);
here = fileparts(mfilename('fullpath'));
stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
writetable(summary,fullfile(here,['gimbal_pitch_tuning_' stamp '.csv']));
end

function out = lastWindow(data,mask,seconds)
part = data(mask,:); tEnd = part.McuTime_s(end);
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
