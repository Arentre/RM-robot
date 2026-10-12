function fit = fit_pitch_sweep_compensation(data, expectedRate)
% Pair opposite, nearly constant velocities at equal IMU angles.
% This cancels Coulomb + viscous friction instead of trusting stiction holds.
assert(license('test','Statistics_Toolbox') && license('test','Signal_Toolbox'), ...
    'Statistics and Signal Processing toolboxes are required.');
if nargin<2, expectedRate=6; end
validateattributes(expectedRate,{'numeric'},{'scalar','positive','finite'});
d=data(data.Phase>=1,:); [t,idx]=unique(d.McuTime_s,'stable'); d=d(idx,:);
Ts=.005; tu=(t(1):Ts:t(end)).';
theta=interp1(t,d.PitchFeedback_deg,tu,'linear');
w=interp1(t,d.PitchRate_deg_s,tu,'linear');
u=interp1(t,d.PitchOutput,tu,'linear');
[b,a]=butter(3,3/(.5/Ts)); wf=filtfilt(b,a,w); acceleration=gradient(wf,Ts);
valid=abs(wf)>.5*expectedRate & abs(wf)<1.5*expectedRate & abs(acceleration)<12;
paired=table(); bins=ceil(min(theta)/2)*2:2:floor(max(theta)/2)*2;
for center=bins
    positive=valid & abs(theta-center)<1 & wf>0;
    negative=valid & abs(theta-center)<1 & wf<0;
    if nnz(positive)<15 || nnz(negative)<15, continue; end
    up=median(u(positive)); down=median(u(negative));
    vp=median(wf(positive)); vn=median(wf(negative));
    if abs(vp+vn)>min(2.5,.4*expectedRate), continue; end
    row=table(center,(up+down)/2,(up-down)/2,vp,vn, ...
        nnz(positive),nnz(negative),'VariableNames', ...
        {'Angle_deg','GravityOutput','MovingFrictionOutput','PositiveRate','NegativeRate', ...
        'PositiveSamples','NegativeSamples'});
    paired=[paired;row]; %#ok<AGROW>
end
if height(paired)<6 || range(paired.Angle_deg)<16
    fit=struct('Reliable',false,'Reason','Insufficient paired constant-velocity excitation', ...
        'Pairs',paired,'PositiveSamples',nnz(valid & wf>0), ...
        'NegativeSamples',nnz(valid & wf<0));
    disp(paired);
    fprintf('Paired gravity rejected: %d bins; valid +%d / -%d samples.\n', ...
        height(paired),fit.PositiveSamples,fit.NegativeSamples);
    return
end
X=[sind(paired.Angle_deg),cosd(paired.Angle_deg)]; y=paired.GravityOutput;
[coef,stats]=robustfit(X,y,'bisquare',4.685,'off');
residual=y-X*coef; amp=norm(coef); phase=atan2d(coef(2),coef(1));
cv=zeros(height(paired),1);
for k=1:height(paired)
    train=true(height(paired),1); train(k)=false;
    c=robustfit(X(train,:),y(train),'bisquare',4.685,'off');
    cv(k)=y(k)-X(k,:)*c;
end
friction=median(paired.MovingFrictionOutput);
reliable=amp<.08 && cond(X)<15 && rms(cv)<.0025 && ...
    range(X*coef)>max(.001,3*rms(residual)) && norm(stats.se)<.5*max(amp,.001);
fit=struct('Reliable',reliable,'Amplitude',amp,'Phase_deg',phase, ...
    'MovingFrictionOutput',friction,'RMSE',rms(residual),'CVRMSE',rms(cv), ...
    'ConditionNumber',cond(X),'CoefficientSE',stats.se,'Pairs',paired);
disp(paired);
fprintf('Paired gravity: A %.6f phase %.2f, friction %.6f, CV RMSE %.6f, accepted=%d\n', ...
    amp,phase,friction,rms(cv),reliable);
end
