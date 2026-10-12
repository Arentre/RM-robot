function data = run_gimbal_pitch_bench_test(port, step_deg)
% Safely run a small Pitch step and return. Support the payload by hand.
if nargin < 2, step_deg = 0.5; end
validateattributes(step_deg,{'numeric'}, ...
    {'scalar','real','finite','positive','<=',1.0});
s = getPort(port);
configureTerminator(s,'LF');
s.Timeout = 0.25;
flush(s,'input');
writeline(s,'STOP');
pause(0.1);
flush(s,'input');
stopGuard = onCleanup(@() safeStop(s));

writeline(s,'ENABLE=1');
pause(0.05);
writeline(s,'PITCH=1');
[a,t0,origin] = collectPhase(s,1.5,0,NaN,NaN);
writeline(s,sprintf('PITREL=%.3f',step_deg));
[b,~,~] = collectPhase(s,3.0,1,t0,origin);
writeline(s,'CENTER');
[c,~,~] = collectPhase(s,3.0,2,t0,origin);
writeline(s,'HOLD');
pause(0.1);
writeline(s,'STOP');

rows = [a;b;c];
data = array2table(rows(:,1:end-1),'VariableNames',telemetryNames());
data.Phase = categorical(rows(:,end),[0 1 2], ...
    {'baseline','positive_step','return_step'});
data.TestTime_s = data.McuTime_s-data.McuTime_s(1);
data.PitchTravel_deg = data.PitchFeedback_deg-origin;

here = fileparts(mfilename('fullpath'));
stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
csvPath = fullfile(here,['gimbal_pitch_bench_' stamp '.csv']);
pngPath = fullfile(here,['gimbal_pitch_bench_' stamp '.png']);
writetable(data,csvPath);
f = figure('Color','w','Name','GM6020 safe Pitch bench test', ...
    'Position',[80 80 1250 850]);
tiledlayout(3,2,'TileSpacing','compact');
nexttile; plot(data.TestTime_s,data.PitchTarget_deg,'k--', ...
    data.TestTime_s,data.PitchFeedback_deg,'r','LineWidth',1.1);
grid on; title('Pitch angle'); ylabel('deg'); legend('Target','Feedback');
nexttile; plot(data.TestTime_s,data.PitchError_deg,'r');
grid on; title('Pitch error'); ylabel('deg');
nexttile; plot(data.TestTime_s,data.PitchRate_deg_s,'r');
grid on; title('Pitch rate'); ylabel('deg/s');
nexttile; plot(data.TestTime_s,data.PitchOutput,'k', ...
    data.TestTime_s,data.PitchActualCurrent_A,'r');
grid on; title('Command / actual current');
legend('Normalized output','Current (A)','Location','best');
nexttile; plot(data.TestTime_s,data.PitchTravel_deg,'r');
yline(5,'r:'); yline(-5,'r:'); grid on;
title('Travel from enable pose'); ylabel('deg'); xlabel('MCU time (s)');
nexttile; plot(data.TestTime_s,data.PitchTemp_C,'r');
grid on; title('Motor temperature'); ylabel('deg C'); xlabel('MCU time (s)');
exportgraphics(f,pngPath,'Resolution',160);
fprintf(['Pitch test complete: peak travel %.2f deg, peak rate %.1f deg/s, ' ...
    'peak current %.3f A.\n'],max(abs(data.PitchTravel_deg)), ...
    max(abs(data.PitchRate_deg_s)),max(abs(data.PitchActualCurrent_A)));
fprintf('Saved %s and %s\n',csvPath,pngPath);
end

function [rows,firstMcuTime,origin] = collectPhase(s,duration_s,phase,firstMcuTime,origin)
rows = zeros(0,42);
t0 = tic; lastPing = tic; lastFrame = tic; enableGrace = 0.30;
while toc(t0) < duration_s
    if toc(lastPing) >= 0.25, writeline(s,'PING'); lastPing = tic; end
    if s.NumBytesAvailable == 0
        if toc(lastFrame) > 0.6
            writeline(s,'STOP'); error('Telemetry timeout; STOP sent.');
        end
        pause(0.002); continue
    end
    line = strtrim(string(readline(s)));
    if ~startsWith(line,'yaw:'), continue; end
    v = str2double(split(extractAfter(line,4),','));
    if ~(numel(v) == 35 || numel(v) == 41) || any(~isfinite(v)), continue; end
    if numel(v) == 35, v(36:41) = NaN; end
    lastFrame = tic;
    if phase == 0 && toc(t0) < enableGrace && (v(12) == 0 || v(22) == 0)
        continue
    end
    if isnan(firstMcuTime), firstMcuTime = v(27); end
    if isnan(origin), origin = v(14); end
    assertSafe(s,v,origin);
    rows(end+1,:) = [v(:).',phase]; %#ok<AGROW>
end
if isempty(rows)
    writeline(s,'STOP');
    error('No valid telemetry; flash the diagnostic firmware first.');
end
end

function assertSafe(s,v,origin)
reason = v(32);
travel = v(14)-origin;
if reason ~= 0 || v(10) ~= 0 || v(12) == 0 || v(22) == 0
    writeline(s,'STOP');
    error('Controller stopped or protected: stop reason %.0f.',reason);
end
if abs(travel) > 5.5 || abs(v(15)) > 120 || abs(v(20)) > 0.101 || ...
        abs(v(29)) > 0.45 || v(31) > 80
    writeline(s,'STOP');
    error(['Pitch safety threshold exceeded; STOP sent. Travel %.2f deg, ' ...
        'rate %.1f deg/s, output %.3f, current %.3f A, temp %.1f C.'], ...
        travel,v(15),v(20),v(29),v(31));
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
