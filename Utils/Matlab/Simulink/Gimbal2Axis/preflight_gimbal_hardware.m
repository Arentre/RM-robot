function report = preflight_gimbal_hardware(port, duration_s)
% 先发 STOP，再只读检查状态，全程不使能电机。
if nargin < 2, duration_s = 3; end
s = getPort(port);
configureTerminator(s,'LF');
s.Timeout = 0.5;
writeline(s,'STOP');
stopGuard = onCleanup(@() safeStop(s));
pause(0.25);
flush(s,'input');
data = capture_gimbal_hardware(s,duration_s);

assert(all(isfinite(data.McuTime_s)), ...
    'Only legacy 26-channel telemetry was found. Flash the updated firmware.');
assert(all(data.Enable == 0), ...
    'Preflight found an enabled control frame; STOP and inspect the firmware.');
assert(max(abs(data.YawOutput)) < 0.02 && max(abs(data.PitchOutput)) < 0.02, ...
    'Motor output is nonzero while stopped. Do not continue.');
assert(max(abs(data.YawRate_deg_s)) < 30 && ...
       max(abs(data.PitchRate_deg_s)) < 30, ...
    'The gimbal moved during the read-only preflight. Secure it and retry.');
assert(all(data.YawTemp_C >= 1 & data.YawTemp_C <= 100) && ...
       all(data.PitchTemp_C >= 1 & data.PitchTemp_C <= 100), ...
    'GM6020 temperature feedback is missing or implausible. Check CAN and IDs.');
assert(max(abs(data.YawActualCurrent_A)) < 0.60 && ...
       max(abs(data.PitchActualCurrent_A)) < 0.60, ...
    'Unexpected current feedback while stopped. Check wiring and scaling.');

dt = diff(data.McuTime_s);
dt = dt(isfinite(dt) & dt > 0);
assert(~isempty(dt), 'MCU timestamps did not advance.');
report = table(height(data),1/median(dt), ...
    median(data.YawTemp_C),median(data.PitchTemp_C), ...
    max(abs(data.YawActualCurrent_A)),max(abs(data.PitchActualCurrent_A)), ...
    'VariableNames',{'Frames','TelemetryRate_Hz','YawTemp_C','PitchTemp_C', ...
    'PeakStoppedYawCurrent_A','PeakStoppedPitchCurrent_A'});
disp(report);
fprintf('Read-only preflight passed. Both axes remained disabled.\n');
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
