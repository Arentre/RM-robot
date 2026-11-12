function summary = evaluate_gimbal_chirp_models(estimationData,validationData)
% 在接受模型前，比较不同预处理方法和模型阶次。
inputModes = ["current","command","friction_compensated","closed_loop"];
cutoffs_Hz = [2.5 3 4 6];
orders = [1 0;2 0;2 1;3 1;3 2];
summary = table();
for inputMode = inputModes
    for cutoff = cutoffs_Hz
        zEst = makeData(estimationData,inputMode,cutoff);
        zVal = makeData(validationData,inputMode,cutoff);
        for orderIndex = 1:size(orders,1)
            np = orders(orderIndex,1); nz = orders(orderIndex,2);
            try
                opt = tfestOptions('Display','off');
                opt.InitialCondition = 'estimate';
                model = tfest(zEst,np,nz,opt);
                reverseModel = tfest(zVal,np,nz,opt);
                [~,forwardFit] = compare(zVal,model);
                [~,reverseFit] = compare(zEst,reverseModel);
                [~,selfFit] = compare(zEst,model);
                stable = isstable(model) && isstable(reverseModel);
                row = table(inputMode,cutoff,np,nz,selfFit(1), ...
                    forwardFit(1),reverseFit(1),stable, ...
                    'VariableNames',{'InputMode','Cutoff_Hz','Poles','Zeros', ...
                    'SelfFit_pct','ForwardValidationFit_pct', ...
                    'ReverseValidationFit_pct','Stable'});
                summary = [summary;row]; %#ok<AGROW>
            catch ME
                warning('evaluate_gimbal_chirp_models:CandidateFailed', ...
                    '%s %.1f Hz P%dZ%d: %s',inputMode,cutoff,np,nz,ME.message);
            end
        end
    end
end
summary.MeanValidationFit_pct = ...
    (summary.ForwardValidationFit_pct+summary.ReverseValidationFit_pct)/2;
summary = sortrows(summary,'MeanValidationFit_pct','descend');
disp(summary(1:min(15,height(summary)),:));
end

function z = makeData(data,inputMode,cutoff_Hz)
phase = string(data.Phase);
baseline = phase == "baseline" & abs(data.YawRate_deg_s) < 1;
active = phase == "chirp";
t = data.McuTime_s(active);
if inputMode == "closed_loop"
    y = unwrap(deg2rad(data.YawFeedback_deg(active)));
    u = unwrap(deg2rad(data.YawTarget_deg(active)));
else
    bias = median(data.YawActualCurrent_A(baseline));
    y = deg2rad(data.YawRate_deg_s(active));
    switch inputMode
        case "current"
            u = data.YawActualCurrent_A(active)-bias;
        case "command"
            u = 3*data.YawOutput(active);
        case "friction_compensated"
            u = data.YawActualCurrent_A(active)-bias-3*data.YawFrictionFF(active);
    end
end
[t,index] = unique(t,'stable'); y = y(index); u = u(index);
Ts = 0.005;
tu = (t(1):Ts:t(end)).';
y = interp1(t,y,tu,'pchip');
u = interp1(t,u,tu,'linear');
y = lowpass(y,cutoff_Hz,1/Ts,'ImpulseResponse','iir', ...
    'Steepness',0.85);
u = lowpass(u,cutoff_Hz,1/Ts,'ImpulseResponse','iir', ...
    'Steepness',0.85);
z = detrend(iddata(y,u,Ts),0);
end
