function [guard,limit] = gimbal_bench_budget(s,axis,requestedLimit)
% 监督试验时临时降低已安装的轴输出上限。
% 使能前读回核对，原设置更低时不提高它。
validateattributes(requestedLimit,{'numeric'}, ...
    {'scalar','real','finite','>=',0.005,'<=',1});
original = gimbal_read_axis_params(s,axis);
limit = min(original(5),requestedLimit);
guard = onCleanup(@() restoreAxis(s,axis,original));
if original(5) > limit
    temporary = original; temporary(5) = limit;
    setAxis(s,axis,temporary);
    actual = gimbal_read_axis_params(s,axis);
    assert(abs(actual(5)-limit) <= 0.0002, ...
        '%s test limit could not be confirmed; trial was not enabled.',axis);
end
end

function setAxis(s,axis,p)
name = [upper(axis) 'PARAM'];
command = sprintf('%s=%.6g,%.6g,%.6g,%.6g,%.6g',name,p);
gimbal_usb_command(s,command,['ack:' name]);
end

function restoreAxis(s,axis,original)
try
    gimbal_usb_command(s,'STOP','ack:STOP');
    setAxis(s,axis,original);
catch ME
    warning('gimbal_bench_budget:RestoreFailed', ...
        'Could not restore %s axis settings: %s',axis,ME.message);
end
end
