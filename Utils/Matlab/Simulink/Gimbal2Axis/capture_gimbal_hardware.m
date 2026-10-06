function data = capture_gimbal_hardware(port, duration_s)
% Record the USB CDC FireWater telemetry from the STM32.
% port may be a serialport object or a COM port name such as "COM7".
if nargin < 2, duration_s = 10; end
validateattributes(duration_s, {'numeric'}, {'scalar','finite','positive'});
if ischar(port) || (isstring(port) && isscalar(port))
    s = serialport(port, 115200);
elseif isobject(port) && isprop(port, 'NumBytesAvailable')
    % In R2024a the public serialport object may report its implementation
    % class as internal.Serialport, so do not rely on isa(port,'serialport').
    s = port;
else
    error('port must be a COM port name or an existing serialport object.');
end
configureTerminator(s, 'LF');
s.Timeout = 1;

names = {'HostTime_s','YawTarget_deg','YawFeedback_deg', ...
    'YawRate_deg_s','YawError_deg','YawControllerError_deg', ...
    'YawRateTarget_deg_s','YawRateError_deg_s','YawOutput', ...
    'YawEncoder_deg','PitchProtect','Mode','Enable', ...
    'PitchTarget_deg','PitchFeedback_deg','PitchRate_deg_s', ...
    'PitchError_deg','PitchControllerError_deg','PitchRateTarget_deg_s', ...
    'PitchRateError_deg_s','PitchOutput','PitchEncoder_deg', ...
    'PitchLadrcEnable','PitchTargetValid','PitchLimitedTarget_deg', ...
    'PitchLimitActive','PitchLadrcOutput','McuTime_s', ...
    'YawActualCurrent_A','PitchActualCurrent_A','YawTemp_C','PitchTemp_C', ...
    'StopReason','YawOrigin_deg','YawTravel_deg','YawFrictionFF'};
samples = zeros(max(1, ceil(duration_s*55)), numel(names));
n = 0;
t0 = tic;
while toc(t0) < duration_s
    if s.NumBytesAvailable == 0
        pause(0.005);
        continue
    end
    try
        line = strtrim(string(readline(s)));
    catch ME
        warning('capture_gimbal_hardware:SerialRead', ...
            'Serial read stopped: %s', ME.message);
        break
    end
    if ~startsWith(line, 'yaw:'), continue; end
    values = str2double(split(extractAfter(line, 4), ','));
    if ~(numel(values) == 26 || numel(values) == 34 || numel(values) == 35) || ...
            any(~isfinite(values)), continue; end
    if numel(values) == 26
        values(27:35,1) = NaN; % compatibility with firmware before bench protocol
    elseif numel(values) == 34
        values(35,1) = NaN; % compatibility with firmware before friction FF
    end
    n = n + 1;
    if n > size(samples, 1), samples(end+100, :) = 0; end %#ok<AGROW>
    samples(n, :) = [toc(t0), values(:).'];
end
if n == 0
    error('No valid yaw: telemetry frames received. Check the COM port and firmware.');
end
data = array2table(samples(1:n, :), 'VariableNames', names);
here = fileparts(mfilename('fullpath'));
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
csvPath = fullfile(here, ['gimbal_hardware_' stamp '.csv']);
pngPath = fullfile(here, ['gimbal_hardware_' stamp '.png']);
writetable(data, csvPath);

time = data.HostTime_s;
timeLabel = 'Host time (s)';
if all(isfinite(data.McuTime_s)) && range(data.McuTime_s) > 0
    time = data.McuTime_s - data.McuTime_s(1);
    timeLabel = 'MCU time (s)';
end
f = figure('Color','w','Name','GM6020 hardware telemetry', ...
    'Position',[80 40 1250 1050]);
tiledlayout(4,2,'TileSpacing','compact');
nexttile; plot(time, data.YawTarget_deg, 'k--', ...
    time, data.YawFeedback_deg, 'b');
grid on; title('Yaw angle'); ylabel('deg'); legend('Target','Feedback');
nexttile; plot(time, data.PitchTarget_deg, 'k--', ...
    time, data.PitchFeedback_deg, 'r');
grid on; title('Pitch angle'); ylabel('deg'); legend('Target','Feedback');
nexttile; plot(time, data.YawError_deg, 'b');
grid on; title('Yaw error'); ylabel('deg');
nexttile; plot(time, data.PitchError_deg, 'r');
grid on; title('Pitch error'); ylabel('deg');
nexttile;
yyaxis left; plot(time, data.YawRate_deg_s, 'b'); ylabel('deg/s');
yyaxis right; plot(time, data.YawOutput, 'k:'); ylabel('Output');
grid on; title('Yaw rate / output'); xlabel(timeLabel);
legend('Rate (deg/s)','Output (normalized)');
nexttile;
yyaxis left; plot(time, data.PitchRate_deg_s, 'r'); ylabel('deg/s');
yyaxis right; plot(time, data.PitchOutput, 'k:'); ylabel('Output');
grid on; title('Pitch rate / output'); xlabel(timeLabel);
legend('Rate (deg/s)','Output (normalized)');
nexttile; plot(time, data.YawActualCurrent_A, 'b', ...
    time, 3*data.YawOutput, 'k:',time,3*data.YawFrictionFF,'m--');
grid on; title('Yaw current'); xlabel(timeLabel); ylabel('A');
legend('Actual torque current','Command equivalent','Friction FF equivalent', ...
    'Location','best');
nexttile; yyaxis left; plot(time, data.YawTravel_deg, 'b'); ylabel('deg');
yyaxis right; plot(time, data.YawTemp_C, 'r'); ylabel('deg C');
grid on; title('Yaw travel / temperature'); xlabel(timeLabel);
exportgraphics(f, pngPath, 'Resolution', 160);
fprintf('Recorded %d valid frames at %.1f Hz (host timestamps).\n', ...
    n, n / data.HostTime_s(end));
fprintf('Saved %s and %s\n', csvPath, pngPath);
end
