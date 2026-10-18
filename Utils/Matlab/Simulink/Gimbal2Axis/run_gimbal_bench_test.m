function data = run_gimbal_bench_test(port, step_deg)
% Run a bounded Yaw step and return using the protected USB bench protocol.
if nargin < 2, step_deg = 30.0; end
validateattributes(step_deg, {'numeric'}, ...
    {'scalar','real','finite','positive','<=',30.0});
s = getPort(port);
configureTerminator(s, 'LF');
s.Timeout = 0.25;
flush(s, 'input');
writeline(s, 'STOP');
pause(0.1);
flush(s, 'input');
stopGuard = onCleanup(@() safeStop(s));

writeline(s, 'ENABLE=1');
[a, t0] = collectPhase(s, 1.5, 0, NaN);
writeline(s, sprintf('YAWREL=%.3f', step_deg));
wakeTime = max(3.0,0.15*step_deg);
[b, ~] = collectPhase(s, wakeTime, 1, t0);
writeline(s, 'CENTER');
[c, ~] = collectPhase(s, wakeTime, 2, t0);
writeline(s, 'HOLD');
pause(0.1);
writeline(s, 'STOP');

rows = [a; b; c];
names = telemetryNames();
data = array2table(rows(:,1:end-1), 'VariableNames', names);
data.Phase = categorical(rows(:,end), [0 1 2], ...
    {'baseline','positive_step','return_step'});
data.TestTime_s = data.McuTime_s - data.McuTime_s(1);

here = fileparts(mfilename('fullpath'));
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
csvPath = fullfile(here, ['gimbal_bench_' stamp '.csv']);
pngPath = fullfile(here, ['gimbal_bench_' stamp '.png']);
writetable(data, csvPath);

f = figure('Color','w','Name','GM6020 safe bench test', ...
    'Position',[80 80 1250 850]);
tiledlayout(3,2,'TileSpacing','compact');
nexttile; plot(data.TestTime_s,data.YawTarget_deg,'k--', ...
    data.TestTime_s,data.YawFeedback_deg,'b','LineWidth',1.1);
grid on; title('Yaw angle'); ylabel('deg'); legend('Target','Feedback');
nexttile; plot(data.TestTime_s,data.YawError_deg,'b');
grid on; title('Yaw error'); ylabel('deg');
nexttile; plot(data.TestTime_s,data.YawRate_deg_s,'b');
grid on; title('Yaw rate'); ylabel('deg/s');
nexttile; plot(data.TestTime_s,data.YawOutput,'k', ...
    data.TestTime_s,data.YawActualCurrent_A,'b');
grid on; title('Command / actual current');
legend('Normalized output','Current (A)','Location','best');
nexttile; plot(data.TestTime_s,data.YawTravel_deg,'b');
yline(35,'r:'); yline(-35,'r:'); grid on;
title('Travel from enable pose'); ylabel('deg'); xlabel('MCU time (s)');
nexttile; plot(data.TestTime_s,data.YawTemp_C,'r');
grid on; title('Motor temperature'); ylabel('deg C'); xlabel('MCU time (s)');
exportgraphics(f,pngPath,'Resolution',160);

positive = data.Phase == 'positive_step';
fprintf(['Bench test complete: peak travel %.2f deg, peak rate %.1f deg/s, ' ...
    'peak current %.3f A, final step error %.3f deg.\n'], ...
    max(abs(data.YawTravel_deg)), max(abs(data.YawRate_deg_s)), ...
    max(abs(data.YawActualCurrent_A)), ...
    mean(data.YawError_deg(find(positive,20,'last'))));
fprintf('Saved %s and %s\n',csvPath,pngPath);
end

function [rows, firstMcuTime] = collectPhase(s, duration_s, phase, firstMcuTime)
rows = zeros(0,44);
t0 = tic;
lastPing = tic;
lastFrame = tic;
enableGrace = 0.30;
while toc(t0) < duration_s
    if toc(lastPing) >= 0.25
        writeline(s,'PING');
        lastPing = tic;
    end
    if s.NumBytesAvailable == 0
        if toc(lastFrame) > 0.6
            writeline(s,'STOP');
            error('Telemetry timeout; STOP sent.');
        end
        pause(0.002);
        continue
    end
    line = strtrim(string(readline(s)));
    if ~startsWith(line,'yaw:'), continue; end
    v = str2double(split(extractAfter(line,4),','));
    if ~(numel(v) == 35 || numel(v) == 41 || numel(v) == 43) || ...
            any(~isfinite(v)), continue; end
    if numel(v) == 35, v(36:43) = NaN; end
    if numel(v) == 41, v(42:43) = NaN; end
    lastFrame = tic;
    % A telemetry line already queued before ENABLE may still report Enable=0.
    if phase == 0 && toc(t0) < enableGrace && v(12) == 0, continue; end
    if isnan(firstMcuTime), firstMcuTime = v(27); end
    assertSafe(s,v);
    rows(end+1,:) = [v(:).',phase]; %#ok<AGROW>
end
if isempty(rows)
    writeline(s,'STOP');
    error('No valid telemetry; flash the diagnostic firmware first.');
end
end

function assertSafe(s,v)
reason = v(32);
if reason ~= 0 || v(10) ~= 0 || v(12) == 0
    writeline(s,'STOP');
    error('Controller stopped or protected: stop reason %.0f.',reason);
end
if abs(v(34)) > 35.5 || abs(v(3)) > 180 || abs(v(8)) > 0.151 || ...
        abs(v(28)) > 0.60 || v(30) > 80
    writeline(s,'STOP');
    error(['Safety threshold exceeded; STOP sent. Travel %.2f deg, ' ...
        'rate %.1f deg/s, output %.3f, current %.3f A, temp %.1f C.'], ...
        v(34),v(3),v(8),v(28),v(30));
end
end

function names = telemetryNames()
names = {'YawTarget_deg','YawFeedback_deg','YawRate_deg_s','YawError_deg', ...
    'YawControllerError_deg','YawRateTarget_deg_s','YawRateError_deg_s', ...
    'YawOutput','YawEncoder_deg','PitchProtect','Mode','Enable', ...
    'PitchTarget_deg','PitchFeedback_deg','PitchRate_deg_s','PitchError_deg', ...
    'PitchControllerError_deg','PitchRateTarget_deg_s','PitchRateError_deg_s', ...
    'PitchOutput','PitchEncoder_deg','PitchLadrcEnable','PitchTargetValid', ...
    'PitchLimitedTarget_deg','PitchLimitActive','PitchLadrcOutput', ...
    'McuTime_s','YawActualCurrent_A','PitchActualCurrent_A','YawTemp_C', ...
    'PitchTemp_C','StopReason','YawOrigin_deg','YawTravel_deg', ...
    'YawFrictionFF','GyroX_deg_s','GyroY_deg_s','GyroZ_deg_s', ...
    'AccelX_g','AccelY_g','AccelZ_g'};
names = [names,{'RollFeedback_deg','BulletSpeed_m_s'}];
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

function safeStop(s)
try
    writeline(s,'STOP');
catch
end
end
