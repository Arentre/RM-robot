function summary = tune_gimbal_friction(port,staticCandidates)
% Tune bounded Yaw breakaway/Coulomb feedforward using protected 2 deg tests.
if nargin < 2, staticCandidates = [0.025 0.035 0.040]; end
validateattributes(staticCandidates,{'numeric'}, ...
    {'vector','real','finite','nonnegative','<=',0.06});
s = getPort(port);
configureTerminator(s,'LF');
summary = table();
for k = 1:numel(staticCandidates)
    staticFF = staticCandidates(k);
    coulombFF = min(0.020,0.60*staticFF);
    writeline(s,sprintf('YFFPARAM=%.6g,%.6g,0.2,5,0.12', ...
        staticFF,coulombFF));
    writeline(s,'YFF=1');
    pause(0.15);
    trial = run_gimbal_bench_test(s,2.0);
    positive = trial.Phase == 'positive_step';
    p = trial(positive,:);
    moving = find(abs(p.YawRate_deg_s) >= 5,1);
    if isempty(moving)
        breakawayDelay_s = Inf;
    else
        breakawayDelay_s = p.McuTime_s(moving)-p.McuTime_s(1);
    end
    startAngle = p.YawFeedback_deg(1);
    direction = sign(mean(p.YawTarget_deg)-startAngle);
    response = direction*(p.YawFeedback_deg-startAngle);
    peakResponse_deg = max(response);
    amplitudeError_deg = abs(peakResponse_deg-2.0);
    overshoot_deg = max(0,peakResponse_deg-2.0);
    tail = p(max(1,height(p)-round(0.4/median(diff(p.McuTime_s)))):end,:);
    finalError_deg = mean(abs(tail.YawError_deg));
    peakRate_deg_s = max(abs(trial.YawRate_deg_s));
    peakCurrent_A = max(abs(trial.YawActualCurrent_A));
    row = table(staticFF,coulombFF,breakawayDelay_s,peakResponse_deg, ...
        amplitudeError_deg,overshoot_deg, ...
        finalError_deg,peakRate_deg_s,peakCurrent_A, ...
        'VariableNames',{'StaticFF','CoulombFF','BreakawayDelay_s', ...
        'PeakResponse_deg','AmplitudeError_deg','Overshoot_deg', ...
        'FinalError_deg','PeakRate_deg_s','PeakCurrent_A'});
    summary = [summary;row]; %#ok<AGROW>
end
summary.Score = summary.BreakawayDelay_s+0.60*summary.AmplitudeError_deg+ ...
    0.20*summary.Overshoot_deg+0.30*summary.FinalError_deg+ ...
    0.003*summary.PeakRate_deg_s+ ...
    0.20*summary.PeakCurrent_A;
[~,best] = min(summary.Score);
bestRow = summary(best,:);
writeline(s,sprintf('YFFPARAM=%.6g,%.6g,0.2,5,0.12', ...
    bestRow.StaticFF,bestRow.CoulombFF));
writeline(s,'YFF=1');
writeline(s,'STOP');
disp(summary);
fprintf('Selected friction FF: static %.4f, Coulomb %.4f normalized output.\n', ...
    bestRow.StaticFF,bestRow.CoulombFF);
here = fileparts(mfilename('fullpath'));
stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
writetable(summary,fullfile(here,['gimbal_friction_tuning_' stamp '.csv']));
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
