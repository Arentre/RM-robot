function result = pitch_hil_trial(port, offsets, durations, cfg)
% 记录闭环 Pitch 试验，角度偏移相对于使能时的姿态。
% 兼容 41、43、47 列遥测，不改 Yaw 调参值；每次退出发送 STOP。
if nargin < 4, cfg = struct(); end
if nargin < 3, durations = 4*ones(size(offsets)); end
validateattributes(offsets,{'numeric'},{'vector','real','finite','nonempty'});
validateattributes(durations,{'numeric'},{'vector','real','finite','positive'});
assert(numel(offsets)==numel(durations),'One duration per target is required.');
if ~isfield(cfg,'Label'), cfg.Label = 'trial'; end
if ~isfield(cfg,'Active'), cfg.Active = true; end
if ~isfield(cfg,'MaxRate'), cfg.MaxRate = 150; end
if ~isfield(cfg,'MaxOutput'), cfg.MaxOutput = 0.125; end
if ~isfield(cfg,'MaxCurrent'), cfg.MaxCurrent = 0.65; end
if ~isfield(cfg,'RestoreOnExit'), cfg.RestoreOnExit = true; end
if ischar(port) || isstring(port), s = serialport(port,115200); else, s = port; end
configureTerminator(s,'LF'); s.Timeout = 0.3;
here = fileparts(mfilename('fullpath'));
stamp = char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
path = fullfile(here,['pitch_hil_' cfg.Label '_' stamp]);
rows = zeros(0,44); rawState = ''; original = struct(); origin = NaN;
hostClock = tic; partial = ''; status = 'incomplete';
guard = makeStopGuard(s); %#ok<NASGU>
try
    command('STOP'); rawState = command('GET');
    original = stateConfig(rawState);
    assert(isfield(original,'Ladrc') && contains(rawState,'pdead='), ...
        'Updated runtime tuning firmware is required.');
    collect(1,-1,false);
    stopped = rows(:,2:42);
    assert(all(stopped(:,12)==0) && all(stopped(:,22)==0) && ...
        max(abs(stopped(:,8)))<0.002 && max(abs(stopped(:,20)))<0.002, ...
        'Stopped motor output is nonzero.');
    assert(max(abs(stopped(:,15)))<10 && max(abs(stopped(:,3)))<10, ...
        'Preflight is moving; do not identify moving/support forces.');
    assert(all(stopped(:,30)>0 & stopped(:,30)<75) && ...
        all(stopped(:,31)>0 & stopped(:,31)<75),'Missing/overheated CAN feedback.');
    if cfg.Active
        applyConfig(cfg);
        command('ENABLE=1'); command('PITCH=1');
        state = command('GET'); origin = scalarField(state,'pit');
        if isfield(cfg,'BaseAngle_deg'), origin = cfg.BaseAngle_deg; end
        assert(abs(origin)<50 && all(abs(origin+offsets)<55), ...
            'Experiment targets must remain inside +/-55 deg.');
        fprintf('Pitch origin %.4f deg; label %s\n',origin,cfg.Label);
        previousTarget=scalarField(state,'pit');
        for k=1:numel(offsets)
            desiredTarget=origin+offsets(k);
            if isfield(cfg,'RampRate_deg_s') && cfg.RampRate_deg_s>0
                ramp=struct('Start',previousTarget,'End',desiredTarget, ...
                    'Duration',abs(desiredTarget-previousTarget)/cfg.RampRate_deg_s);
                collect(durations(k)+ramp.Duration,k,true,ramp);
                command(sprintf('PITABS=%.5f',desiredTarget));
            else
                command(sprintf('PITABS=%.5f',desiredTarget));
                collect(durations(k),k,true);
            end
            previousTarget=desiredTarget;
            part = rows(rows(:,43)==k,:);
            assert(size(part,1)>=15,'Not enough telemetry in phase %d.',k);
        end
    else
        collect(durations(1),0,false);
    end
    command('STOP'); finalState = command('GET');
    assert(scalarField(finalState,'en')==0 && scalarField(finalState,'serial')==0, ...
        'STOP not confirmed.');
    if cfg.RestoreOnExit, applyConfig(original); command('STOP'); end
    restoredState = command('GET');
    status = 'complete'; result = makeResult();
    result.InitialState = rawState; result.FinalState = finalState;
    result.RestoredState = restoredState;
    save([path '.mat'],'result','cfg');
    fprintf('Saved %s.csv (%d frames); STOP confirmed.\n',path,height(result.Data));
catch ME
    status = ['failed: ' ME.message];
    emergencyStop(s);
    try
        command('STOP');
        if cfg.RestoreOnExit && ~isempty(fieldnames(original)), applyConfig(original); end
        command('STOP');
    catch restoreError
        fprintf(2,'Restore failed: %s\n',restoreError.message);
    end
    if ~isempty(rows)
        saved = makeResult(); save([path '.mat'],'saved','cfg','rawState');
    end
    rethrow(ME);
end

    function out = makeResult()
        names = {'HostTime_s','YawTarget_deg','YawFeedback_deg','YawRate_deg_s', ...
            'YawError_deg','YawControllerError_deg','YawRateTarget_deg_s', ...
            'YawRateError_deg_s','YawOutput','YawEncoder_deg','PitchProtect', ...
            'Mode','Enable','PitchTarget_deg','PitchFeedback_deg','PitchRate_deg_s', ...
            'PitchError_deg','PitchControllerError_deg','PitchRateTarget_deg_s', ...
            'PitchRateError_deg_s','PitchOutput','PitchEncoder_deg','PitchLadrcEnable', ...
            'PitchTargetValid','PitchLimitedTarget_deg','PitchLimitActive', ...
            'PitchLadrcOutput','McuTime_s','YawActualCurrent_A','PitchActualCurrent_A', ...
            'YawTemp_C','PitchTemp_C','StopReason','YawOrigin_deg','YawTravel_deg', ...
            'YawFrictionFF','GyroX_deg_s','GyroY_deg_s','GyroZ_deg_s', ...
            'AccelX_g','AccelY_g','AccelZ_g','Phase','Origin_deg'};
        out = struct('Data',array2table(rows,'VariableNames',names), ...
            'Origin_deg',origin,'Path',path,'Status',status,'Config',cfg);
        writetable(out.Data,[path '.csv']);
    end

    function reply = command(cmd)
        writeline(s,cmd); expected = extractBefore(string(cmd)+'=', '=');
        if strcmp(cmd,'ENABLE=0'), expected = 'STOP'; end
        deadline = tic;
        while toc(deadline)<2
            [line,~] = nextLine();
            if startsWith(line,'err:'), error('pitch_hil:Command','%s: %s',cmd,line); end
            if startsWith(line,'ack:'+expected)
                reply = char(line); return
            end
        end
        error('pitch_hil:Ack','No acknowledgement for %s.',cmd);
    end

    function [line,v] = nextLine()
        line = ""; v = [];
        q = find(partial==newline,1);
        if isempty(q)
            if s.NumBytesAvailable==0, pause(0.001); return; end
            partial = [partial char(read(s,s.NumBytesAvailable,'uint8'))];
            q = find(partial==newline,1);
        end
        if isempty(q), return; end
        line = string(strtrim(partial(1:q))); partial = partial(q+1:end);
        if startsWith(line,'yaw:')
            v = str2double(split(extractAfter(line,4),','));
            if ~ismember(numel(v),[41 43 47]) || any(~isfinite(v))
                v=[];
            else
                v=v(1:41);
            end
        end
    end

    function collect(seconds,phase,active,ramp)
        timer = tic; lastFrame = tic; lastMcu = -Inf;
        lastRamp=-Inf;
        while toc(timer)<seconds
            if nargin>=4 && toc(timer)-lastRamp>=0.05
                fraction=min(1,toc(timer)/max(ramp.Duration,eps));
                reference=ramp.Start+fraction*(ramp.End-ramp.Start);
                writeline(s,sprintf('PITABS=%.5f',reference));
                lastRamp=toc(timer);
            end
            [line,v] = nextLine();
            if startsWith(line,'err:'), error('pitch_hil:RampCommand','%s',line); end
            if isempty(v)
                if toc(lastFrame)>0.7, error('pitch_hil:Telemetry','Telemetry lost.'); end
                continue
            end
            if v(27)<=lastMcu, continue; end
            lastMcu=v(27); lastFrame=tic;
            rows(end+1,:)=[toc(hostClock),v(:).',phase,origin]; %#ok<AGROW>
            if active
                if v(12)==0 || v(22)==0 || v(32)~=0 || v(10)~=0
                    error('pitch_hil:Stopped','Controller stopped; reason %.0f.',v(32));
                end
                if abs(v(14))>57 || abs(v(15))>cfg.MaxRate || ...
                        abs(v(20))>cfg.MaxOutput || abs(v(29))>cfg.MaxCurrent || ...
                        v(31)>75 || abs(v(14)-v(13))>max(6,max(abs(diff([0 offsets])))+3)
                    error('pitch_hil:Motion','Trial bounds exceeded: angle %.2f, rate %.1f, output %.4f, current %.3f.', ...
                        v(14),v(15),v(20),v(29));
                end
            end
        end
    end

    function applyConfig(c)
        if isfield(c,'OutputFilterHz'), command(sprintf('PFILTER=%.6g',c.OutputFilterHz)); end
        if isfield(c,'Ladrc'), command(sprintf('PPARAM=%.6g,%.6g,%.6g,%.6g,%.6g',c.Ladrc)); end
        if isfield(c,'Deadband'), command(sprintf('PDEAD=%.6g',c.Deadband)); end
        if isfield(c,'Friction'), command(sprintf('PFFPARAM=%.6g,%.6g,%.6g,%.6g,%.6g',c.Friction)); end
        if isfield(c,'FrictionEnable'), command(sprintf('PFF=%d',c.FrictionEnable)); end
        if isfield(c,'Gravity'), command(sprintf('PGRAVPARAM=%.6g,%.6g',c.Gravity)); end
        if isfield(c,'GravityEnable'), command(sprintf('PGRAV=%d',c.GravityEnable)); end
    end

end

function guard = makeStopGuard(s)
% 在试验的共享嵌套作用域之外捕获串口句柄。
guard = onCleanup(@() emergencyStop(s));
end

function emergencyStop(s)
try, writeline(s,'STOP'); catch, end
end

function c = stateConfig(text)
c = struct();
v = vectorField(text,'pp',5); if ~isempty(v), c.Ladrc=v; end
v = vectorField(text,'pff',6);
if ~isempty(v), c.FrictionEnable=v(1); c.Friction=v(2:6); end
v = vectorField(text,'pgrav',3);
if ~isempty(v), c.GravityEnable=v(1); c.Gravity=v(2:3); end
v = vectorField(text,'pdead',1); if ~isempty(v), c.Deadband=v; end
v = vectorField(text,'pfilter',1); if ~isempty(v), c.OutputFilterHz=v; end
end

function v = vectorField(text,name,count)
token = regexp(text,[name '=([-+0-9.eE,]+)'],'tokens','once');
v = []; if isempty(token), return; end
allValues = str2double(split(string(token{1}),','));
if numel(allValues)>=count && all(isfinite(allValues(1:count)))
    v = allValues(1:count).';
end
end

function v = scalarField(text,name)
values = vectorField(text,name,1); assert(~isempty(values),'Missing GET field %s.',name);
v = values(1);
end
