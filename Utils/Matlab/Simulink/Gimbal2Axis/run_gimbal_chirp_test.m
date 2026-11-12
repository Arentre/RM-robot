function [data,meta] = run_gimbal_chirp_test(port,direction,outputLimit)
% 采集受保护的闭环 Yaw 扫频数据，供独立辨识使用。
% 固件以 5° 幅值、0.2 至 2.0 Hz 生成参考目标。
% 每组扫频持续 20 秒，试验始终监测行程、速度和电流。
if nargin < 2, direction = "up"; end
if nargin < 3, outputLimit = 0.45; end
direction = string(validatestring(char(direction),{'up','down'}));
s = getPort(port);
configureTerminator(s,'LF');
s.Timeout = 0.25;
flush(s,'input');
writeline(s,'STOP');
pause(0.1);
flush(s,'input');
stopGuard = onCleanup(@() safeStop(s));

amplitude_deg = 5.0;
duration_s = 20.0;
if direction == "up"
    f0_Hz = 0.2; f1_Hz = 2.0;
else
    f0_Hz = 2.0; f1_Hz = 0.2;
end
commandPeriod_s = 0; % 参考信号由 500 Hz 固件控制任务生成。

gimbal_usb_command(s,'STOP','ack:STOP');
[budgetGuard,limit] = gimbal_bench_budget(s,'Y',outputLimit);
gimbal_usb_command(s,'ENABLE=YAW','ack:ENABLE,');
[baseline,origin_deg] = collectFixed(s,2.0,0,NaN,limit);
gimbal_usb_command(s,"YCHIRP="+upper(direction),"ack:YCHIRP=");
[excitation,~] = collectFixed(s,duration_s+0.2,1,origin_deg,limit);
gimbal_usb_command(s,'CENTER','ack:CENTER');
[recovery,~] = collectFixed(s,3.0,2,origin_deg,limit);
gimbal_usb_command(s,'HOLD','ack:HOLD');
pause(0.1);
writeline(s,'STOP');

rows = [baseline;excitation;recovery];
data = array2table(rows(:,1:41),'VariableNames',telemetryNames());
data.Phase = categorical(rows(:,end),[0 1 2], ...
    {'baseline','chirp','recovery'});
data.TestTime_s = data.McuTime_s-data.McuTime_s(1);
meta = struct('Direction',direction,'Amplitude_deg',amplitude_deg, ...
    'StartFrequency_Hz',f0_Hz,'EndFrequency_Hz',f1_Hz, ...
    'Duration_s',duration_s,'CommandPeriod_s',commandPeriod_s, ...
    'Origin_deg',origin_deg);

here = fileparts(mfilename('fullpath'));
stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
base = ['gimbal_chirp_' char(direction) '_' stamp];
csvPath = fullfile(here,[base '.csv']);
matPath = fullfile(here,[base '.mat']);
pngPath = fullfile(here,[base '.png']);
writetable(data,csvPath);
save(matPath,'data','meta');

f = figure('Color','w','Name','GM6020 Yaw chirp', ...
    'Position',[80 60 1250 900]);
tiledlayout(4,1,'TileSpacing','compact');
nexttile; plot(data.TestTime_s,data.YawTarget_deg,'k--', ...
    data.TestTime_s,data.YawFeedback_deg,'b');
grid on; ylabel('deg'); legend('Target','Feedback'); title('Yaw chirp');
nexttile; plot(data.TestTime_s,data.YawRate_deg_s,'b');
grid on; ylabel('deg/s'); title('Angular rate');
nexttile; plot(data.TestTime_s,data.YawActualCurrent_A,'b', ...
    data.TestTime_s,3*data.YawOutput,'k:');
grid on; ylabel('A'); legend('Actual current','Command equivalent');
nexttile; plot(data.TestTime_s,data.YawTravel_deg,'b', ...
    data.TestTime_s,data.YawFrictionFF,'m');
grid on; ylabel('deg / output'); xlabel('MCU time (s)');
legend('Travel','Friction FF');
exportgraphics(f,pngPath,'Resolution',160);

fprintf(['Chirp %s complete: travel %.2f deg, rate %.1f deg/s, ' ...
    'current %.3f A, output %.3f, stop reason %.0f.\n'],direction, ...
    max(abs(data.YawTravel_deg)),max(abs(data.YawRate_deg_s)), ...
    max(abs(data.YawActualCurrent_A)),max(abs(data.YawOutput)), ...
    max(data.StopReason));
fprintf('Saved %s\n',csvPath);
end

function [rows,origin_deg] = collectFixed(s,duration_s,phase,origin_deg,limit)
rows = zeros(0,42);
t0 = tic; lastFrame = tic;
while toc(t0) < duration_s
    [v,lastFrame] = nextFrame(s,lastFrame);
    if isempty(v), continue; end
    if phase == 0 && toc(t0) < 0.3 && v(12) == 0, continue; end
    assertSafe(s,v,limit);
    if isnan(origin_deg), origin_deg = mod(v(33),360); end
    rows(end+1,:) = [v(:).',phase]; %#ok<AGROW>
end
assert(~isempty(rows),'No valid telemetry was collected.');
end

function [v,lastFrame] = nextFrame(s,lastFrame)
v = [];
if toc(lastFrame) > 0.6
    writeline(s,'STOP');
    error('No valid telemetry for 0.6 s; STOP sent.');
end
if s.NumBytesAvailable == 0
    pause(0.001);
    return
end
line = strtrim(string(readline(s)));
if ~startsWith(line,'yaw:'), return; end
candidate = str2double(split(extractAfter(line,4),','));
if ~(numel(candidate) == 35 || numel(candidate) == 41 || numel(candidate) == 43 || numel(candidate) == 47) || ...
        any(~isfinite(candidate)), return; end
if numel(candidate) == 35, candidate(36:41) = NaN; end
v = candidate(1:41);
lastFrame = tic;
end

function assertSafe(s,v,limit)
if v(32) ~= 0 || v(10) ~= 0 || v(12) == 0 || v(22) ~= 0 || ...
        abs(v(34)) > 12 || abs(v(3)) > 150 || abs(v(8)) > limit+0.001 || ...
        abs(v(28)) > 1.60 || v(30) > 80
    writeline(s,'STOP');
    error(['Chirp safety threshold exceeded; STOP sent. reason %.0f, ' ...
        'travel %.2f deg, rate %.1f deg/s, output %.3f, current %.3f A.'], ...
        v(32),v(34),v(3),v(8),v(28));
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
