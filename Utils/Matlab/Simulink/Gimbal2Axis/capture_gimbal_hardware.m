function data = capture_gimbal_hardware(port, duration_s)
% Record the existing 50 Hz USB CDC FireWater telemetry from the STM32.
% port may be a serialport object or a COM port name such as "COM7".
if nargin < 2, duration_s = 10; end
validateattributes(duration_s, {'numeric'}, {'scalar','finite','positive'});
if isa(port, 'serialport')
    s = port;
else
    s = serialport(port, 115200);
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
    'PitchLimitActive','PitchLadrcOutput'};
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
    if numel(values) ~= 26 || any(~isfinite(values)), continue; end
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

f = figure('Color','w','Name','GM6020 hardware telemetry', ...
    'Position',[80 80 1250 850]);
tiledlayout(3,2,'TileSpacing','compact');
nexttile; plot(data.HostTime_s, data.YawTarget_deg, 'k--', ...
    data.HostTime_s, data.YawFeedback_deg, 'b');
grid on; title('Yaw angle'); ylabel('deg'); legend('Target','Feedback');
nexttile; plot(data.HostTime_s, data.PitchTarget_deg, 'k--', ...
    data.HostTime_s, data.PitchFeedback_deg, 'r');
grid on; title('Pitch angle'); ylabel('deg'); legend('Target','Feedback');
nexttile; plot(data.HostTime_s, data.YawError_deg, 'b');
grid on; title('Yaw error'); ylabel('deg');
nexttile; plot(data.HostTime_s, data.PitchError_deg, 'r');
grid on; title('Pitch error'); ylabel('deg');
nexttile;
yyaxis left; plot(data.HostTime_s, data.YawRate_deg_s, 'b'); ylabel('deg/s');
yyaxis right; plot(data.HostTime_s, data.YawOutput, 'k:'); ylabel('Output');
grid on; title('Yaw rate / output'); xlabel('Host time (s)');
legend('Rate (deg/s)','Output (normalized)');
nexttile;
yyaxis left; plot(data.HostTime_s, data.PitchRate_deg_s, 'r'); ylabel('deg/s');
yyaxis right; plot(data.HostTime_s, data.PitchOutput, 'k:'); ylabel('Output');
grid on; title('Pitch rate / output'); xlabel('Host time (s)');
legend('Rate (deg/s)','Output (normalized)');
exportgraphics(f, pngPath, 'Resolution', 160);
fprintf('Recorded %d valid frames at %.1f Hz (host timestamps).\n', ...
    n, n / data.HostTime_s(end));
fprintf('Saved %s and %s\n', csvPath, pngPath);
end
