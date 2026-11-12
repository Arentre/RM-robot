function metrics = pitch_hil_metrics(data, outputLimit)
% 各阶段分别检查到位和稳定保持条件。
if nargin < 2, outputLimit = 0.10; end
phases = unique(data.Phase(data.Phase>=1),'stable'); metrics = table();
for phase = phases.'
    p = data(data.Phase==phase,:);
    t = p.McuTime_s-p.McuTime_s(1); error = p.PitchTarget_deg-p.PitchFeedback_deg;
    tail = t>=t(end)-1.0;
    amplitude = p.PitchTarget_deg(end)-p.PitchFeedback_deg(1);
    response = sign(amplitude)*(p.PitchFeedback_deg-p.PitchFeedback_deg(1));
    if abs(amplitude)<0.6, overshoot=NaN; rise=NaN;
    else
        overshoot=max(0,max(response)-abs(amplitude));
        i10=find(response>=0.1*abs(amplitude),1); i90=find(response>=0.9*abs(amplitude),1);
        rise=NaN; if ~isempty(i10) && ~isempty(i90), rise=t(i90)-t(i10); end
    end
    outside=find(abs(error)>0.55,1,'last');
    settling=0;
    if ~isempty(outside)
        if outside==numel(t), settling=Inf; else, settling=t(outside+1); end
    end
    peakToPeak=range(p.PitchFeedback_deg(tail));
    rateRms=rms(p.PitchRate_deg_s(tail));
    error95=prctile(abs(error(tail)),95);
    outputStd=std(p.PitchOutput(tail));
    saturation=mean(abs(p.PitchLadrcOutput)>=0.98*outputLimit);
    stable=error95<=0.55 && peakToPeak<=0.60 && rateRms<=2.0 && ...
        outputStd<=0.010 && settling<min(2.5,t(end)-0.8) && saturation<0.02 && ...
        (isnan(overshoot) || overshoot<=max(0.6,0.10*abs(amplitude)));
    row=table(phase,amplitude,rise,settling,overshoot,error95,peakToPeak,rateRms, ...
        outputStd,saturation,max(abs(p.PitchRate_deg_s)),max(abs(p.PitchActualCurrent_A)),stable, ...
        'VariableNames',{'Phase','Step_deg','Rise10to90_s','Settling_s','Overshoot_deg', ...
        'TailError95_deg','TailPeakToPeak_deg','TailRateRms_deg_s','TailOutputStd', ...
        'SaturationFraction','PeakRate_deg_s','PeakCurrent_A','Stable'});
    metrics=[metrics;row]; %#ok<AGROW>
end
disp(metrics);
end
