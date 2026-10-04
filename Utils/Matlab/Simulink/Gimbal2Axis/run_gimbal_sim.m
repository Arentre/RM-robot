function out = run_gimbal_sim()
% Run the numerical gimbal model and save curves and summary metrics.
here = fileparts(mfilename('fullpath'));
model = 'gimbal_2axis';
if ~isfile(fullfile(here, [model '.slx']))
    build_gimbal_sim();
end
load_system(fullfile(here, [model '.slx']));
out = sim(model);
P = getVariable(get_param(model, 'ModelWorkspace'), 'P');

axisName = {'Yaw', 'Pitch'};
angleName = {'yaw_angle', 'pitch_angle'};
targetName = {'yaw_target', 'pitch_target'};
rateName = {'yaw_rate', 'pitch_rate'};
currentName = {'yaw_current', 'pitch_current'};
commandName = {'yaw_command', 'pitch_command'};
disturbanceName = {'yaw_disturbance', 'pitch_disturbance'};
targetBlock = {'Yaw target', 'Pitch target'};
disturbanceBlock = {'Yaw disturbance', 'Pitch disturbance'};

f = figure('Color', 'w', 'Name', 'GM6020 two-axis response', ...
    'Position', [80 40 1400 1400], 'Visible', 'off');
tiledlayout(5, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
metrics = table();

for k = 1:2
    angle = out.get(angleName{k});
    t = angle.Time(:);
    theta = angle.Data(:);
    target = sampleAt(out.get(targetName{k}), t);
    rate = sampleAt(out.get(rateName{k}), t);
    current = sampleAt(out.get(currentName{k}), t);
    command = sampleAt(out.get(commandName{k}), t);
    disturbance = sampleAt(out.get(disturbanceName{k}), t);
    errorDeg = rad2deg(target - theta);
    if k == 1
        errorDeg = rad2deg(atan2(sin(target - theta), cos(target - theta)));
    end
    motorTorque = P.Kt * current;

    nexttile(k);
    plot(t, rad2deg(target), 'k--', t, rad2deg(theta), 'b', 'LineWidth', 1.2);
    grid on; title([axisName{k} ' angle']); ylabel('Angle (deg)');
    legend('Target', 'Feedback', 'Location', 'best');

    nexttile(k+2);
    plot(t, errorDeg, 'LineWidth', 1.2); yline(0, 'k:');
    grid on; title([axisName{k} ' tracking error']); ylabel('Error (deg)');

    nexttile(k+4);
    plot(t, rad2deg(rate), 'LineWidth', 1.2); yline(0, 'k:');
    grid on; title([axisName{k} ' angular rate']); ylabel('Rate (deg/s)');

    nexttile(k+6);
    plot(t, current, 'LineWidth', 1.2); yline(0, 'k:');
    grid on; title([axisName{k} ' modeled motor current']);
    ylabel('Current (A)');

    nexttile(k+8);
    plot(t, motorTorque, t, disturbance, '--', 'LineWidth', 1.2);
    grid on; title([axisName{k} ' torque']); ylabel('Torque (N m)');
    xlabel('Time (s)');
    legend('Kt x current', 'External disturbance', 'Location', 'best');

    targetTime = str2double(get_param([model '/' targetBlock{k}], 'Time'));
    disturbanceTime = str2double(get_param([model '/' disturbanceBlock{k}], 'Time'));
    testWindow = t >= targetTime & t < disturbanceTime;
    targetStep = target(find(t >= targetTime, 1));
    startAngle = theta(find(t < targetTime, 1, 'last'));
    amplitude = targetStep - startAngle;
    rise90 = NaN;
    settling5 = NaN;
    overshoot = NaN;
    if any(testWindow) && abs(amplitude) > 1e-9
        tw = t(testWindow);
        response = (theta(testWindow) - startAngle) / amplitude;
        reached = find(response >= 0.9, 1);
        if ~isempty(reached), rise90 = tw(reached) - targetTime; end
        overshoot = max(0, (max(response)-1) * 100);
        within = abs(response-1) <= 0.05;
        settled = find(~flipud(cummax(flipud(~within))), 1);
        if ~isempty(settled), settling5 = tw(settled) - targetTime; end
    end
    metrics = [metrics; table(string(axisName{k}), errorDeg(end), ...
        sqrt(mean(errorDeg(testWindow).^2)), max(abs(errorDeg(testWindow))), ...
        max(abs(rad2deg(rate))), max(abs(command)), max(abs(current)), ...
        max(abs(motorTorque)), ...
        rise90, settling5, overshoot, ...
        'VariableNames', {'Axis','FinalError_deg','RmsErrorBeforeDisturbance_deg', ...
        'PeakErrorBeforeDisturbance_deg','PeakRate_deg_s','PeakCommand', ...
        'PeakCurrent_A', ...
        'PeakMotorTorque_Nm','RiseTime90_s','SettlingTime5pct_s', ...
        'Overshoot_pct'})]; %#ok<AGROW>
end

exportgraphics(f, fullfile(here, 'gimbal_response.png'), 'Resolution', 160);
close(f);
writetable(metrics, fullfile(here, 'gimbal_metrics.csv'));
disp(metrics);
fprintf('Saved %s and %s\n', fullfile(here, 'gimbal_response.png'), ...
    fullfile(here, 'gimbal_metrics.csv'));
end

function values = sampleAt(signal, time)
values = interp1(signal.Time(:), signal.Data(:), time, 'previous', 'extrap');
end
