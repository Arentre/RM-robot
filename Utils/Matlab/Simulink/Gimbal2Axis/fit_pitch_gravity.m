function fit = fit_pitch_gravity(input)
% Bidirectional settled holding output -> sin/cos gravity in controller units.
% Robust regression (Statistics Toolbox), not an assumed inertia/gravity model.
assert(license('test','Statistics_Toolbox'),'Statistics Toolbox is required.');
if isstruct(input), d=input.Data; elseif istable(input), d=input; else, d=readtable(input); end
active=d.Phase>=1 & abs(d.PitchLadrcOutput)>0.0005;
outputSign=sign(median(d.PitchOutput(active)./d.PitchLadrcOutput(active)));
assert(isfinite(outputSign) && outputSign~=0,'Cannot determine actuator direction.');
phases=unique(d.Phase(d.Phase>=1),'stable'); holds=table();
for p=phases.'
    q=d(d.Phase==p,:); tail=q.McuTime_s>=q.McuTime_s(end)-1.2;
    movingRms=rms(q.PitchRate_deg_s(tail));
    error95=prctile(abs(q.PitchTarget_deg(tail)-q.PitchFeedback_deg(tail)),95);
    v=q(tail & abs(q.PitchRate_deg_s)<1.5,:);
    if height(v)<30 || movingRms>2 || error95>0.55, continue; end
    row=table(p,median(v.PitchTarget_deg),median(v.PitchFeedback_deg), ...
        outputSign*mean(v.PitchOutput),std(v.PitchOutput)/sqrt(height(v)), ...
        mean(v.PitchActualCurrent_A),movingRms,error95, ...
        'VariableNames',{'Phase','Target_deg','Angle_deg','HoldingOutput', ...
        'MeanOutputSE','HoldingCurrent_A','RateRms_deg_s','Error95_deg'});
    holds=[holds;row]; %#ok<AGROW>
end
assert(height(holds)>=4,'Need at least four settled holding samples.');
theta=deg2rad(holds.Angle_deg);
X=[sin(theta) cos(theta)]; y=holds.HoldingOutput;
[coef,stats]=robustfit(X,y,'bisquare',4.685,'off');
prediction=X*coef; residual=y-prediction;
amplitude=norm(coef); phase=rad2deg(atan2(coef(2),coef(1)));
% Leave-one-target-out validation prevents a narrow noisy scan looking perfect.
targets=unique(round(holds.Target_deg,2)); cv=[]; hysteresis=[];
for target=targets.'
    mask=abs(holds.Target_deg-target)<0.05;
    if nnz(~mask)>=4
        c=robustfit(X(~mask,:),y(~mask),'bisquare',4.685,'off');
        cv=[cv;y(mask)-X(mask,:)*c]; %#ok<AGROW>
    end
    if nnz(mask)>=2, hysteresis(end+1)=range(y(mask)); end %#ok<AGROW>
end
rmse=rms(residual); cvRmse=rms(cv); coefficientSE=stats.se;
signalRange=range(prediction); worstHysteresis=max([0 hysteresis]);
reliable=height(holds)>=8 && range(holds.Angle_deg)>=18 && ...
    amplitude<=0.08 && cond(X)<20 && cvRmse<0.0025 && ...
    worstHysteresis<0.006 && signalRange>max(0.001,3*rmse) && ...
    norm(coefficientSE)<0.5*max(amplitude,0.001);
fit=struct('Reliable',reliable,'Amplitude',amplitude,'Phase_deg',phase, ...
    'SinCoefficient',coef(1),'CosCoefficient',coef(2),'CoefficientSE',coefficientSE, ...
    'OutputSign',outputSign,'RMSE',rmse,'CrossValidationRMSE',cvRmse, ...
    'MaxApproachHysteresis',worstHysteresis,'ConditionNumber',cond(X), ...
    'SignalRange',signalRange,'Holds',holds,'Prediction',prediction);
disp(holds);
fprintf('Gravity A=%.6f phase=%.3f deg; CV RMSE %.6f; hysteresis %.6f; accepted=%d\n', ...
    amplitude,phase,cvRmse,worstHysteresis,reliable);
end
