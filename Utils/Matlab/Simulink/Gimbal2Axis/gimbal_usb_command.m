function line = gimbal_usb_command(s,command,ack)
% 忽略穿插的 FireWater 遥测，只等待对应命令的明确回包。
writeline(s,command);
deadline = tic;
while toc(deadline) < 2
    if s.NumBytesAvailable == 0
        pause(0.002);
        continue
    end
    line = strtrim(char(readline(s)));
    if startsWith(line,ack), return; end
    if startsWith(line,'err:')
        error('gimbal_usb_command:Rejected','%s: %s',command,line);
    end
end
error('gimbal_usb_command:Timeout','No %s reply to %s.',ack,command);
end
