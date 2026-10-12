function result = identify_pitch_hil(data)
% Independent closed-loop TF validation plus integral physical cross-check.
assert(license('test','Identification_Toolbox') && ...
    license('test','Optimization_Toolbox') && license('test','Signal_Toolbox'), ...
    'System Identification, Optimization, Signal Processing toolboxes required.');
d=data(data.Phase>=1,:); [t,idx]=unique(d.McuTime_s,'stable'); d=d(idx,:);
Ts=0.005; tu=(t(1):Ts:t(end)).';
angle=interp1(t,deg2rad(d.PitchFeedback_deg),tu,'linear');
w=interp1(t,deg2rad(d.PitchRate_deg_s),tu,'linear');
u=interp1(t,d.PitchOutput,tu,'linear');
target=interp1(t,deg2rad(d.PitchTarget_deg),tu,'previous');
phase=interp1(t,d.Phase,tu,'previous');
[bf,af]=butter(3,12/(0.5/Ts)); wf=filtfilt(bf,af,w);
% Integral windows avoid differentiating the measured gyro.
n=round(.12/Ts); q=(1:3:numel(tu)-n).'; A=zeros(numel(q),5); y=zeros(numel(q),1);
moving=false(size(q)); split=false(size(q));
for k=1:numel(q)
    r=q(k):q(k)+n;
    A(k,:)=[trapz(tu(r),u(r)), -trapz(tu(r),wf(r)), ...
        -trapz(tu(r),tanh(wf(r)/deg2rad(2))), ...
        -trapz(tu(r),sin(angle(r))),-trapz(tu(r),cos(angle(r)))];
    y(k)=wf(r(end))-wf(r(1));
    moving(k)=max(abs(wf(r)))>deg2rad(3) && numel(unique(phase(r)))==1;
    split(k)=mod(phase(r(1)),2)==0;
end
est=moving & split; val=moving & ~split;
assert(nnz(est)>30 && nnz(val)>30,'Insufficient bidirectional excitation.');
scale=sqrt(mean(A(est,:).^2)); As=A./scale;
opts=optimoptions('lsqlin','Display','off');
lower=[10 0 0 -100 -100].'; upper=[2000 300 300 100 100].';
coef=lsqlin(As(est,:),y(est),[],[],[],[],lower.*scale.',upper.*scale.',[],opts)./scale.';
prediction=A(val,:)*coef; relRmse=rms(y(val)-prediction)/max(rms(y(val)),eps);
result=struct('PhysicalB0',coef(1),'ViscousRate',coef(2), ...
    'CoulombCommand',coef(3)/coef(1),'GravityAmplitude',norm(coef(4:5))/coef(1), ...
    'GravityPhase_deg',rad2deg(atan2(coef(5),coef(4))), ...
    'PhysicalValidationRelativeRMSE',relRmse,'PhysicalCondition',cond(As(est,:)), ...
    'PhysicalReliable',relRmse<.25 && cond(As(est,:))<50 && ...
        coef(1)>11 && coef(1)<1999,'SampleTime_s',Ts);
% Feedback-loop reference->angle ID is safer to interpret than a biased
% closed-loop torque->rate tfest. Keep it separate from the physical fit.
mid=floor(numel(tu)/2);
zEst=iddata(angle(1:mid),target(1:mid),Ts);
zVal=iddata(angle(mid+1:end),target(mid+1:end),Ts);
op=tfestOptions('Display','off'); op.InitialCondition='estimate';
op.EnforceStability=true;
models=cell(1,2); fits=zeros(1,2);
for k=1:2
    models{k}=tfest(zEst,k+1,0,op);
    [~,f]=compare(zVal,models{k}); fits(k)=f(1);
end
[~,best]=max(fits); model=models{best};
result.ClosedLoopValidationFit_pct=fits(best);
result.ClosedLoopModel=model; result.ClosedLoopPoles=pole(model);
result.ClosedLoopDamping=damp(model);
result.ClosedLoopBandwidth_Hz=bandwidth(model)/(2*pi);
fprintf('Physical b0 %.2f, friction %.6f, gravity %.6f phase %.1f; validation relative RMSE %.3f, accepted=%d\n', ...
    result.PhysicalB0,result.CoulombCommand,result.GravityAmplitude, ...
    result.GravityPhase_deg,relRmse,result.PhysicalReliable);
fprintf('System Identification Toolbox independent closed-loop fits: 2p %.1f%% / 3p %.1f%%\n',fits);
% Signal Processing Toolbox: inspect the worst settled-phase oscillation.
worstRms=-Inf; spectrum=struct();
for p=unique(phase).'
    indices=find(phase==p); indices=indices(tu(indices)>=tu(indices(end))-1);
    if numel(indices)<40, continue; end
    rateRms=rms(w(indices));
    if rateRms>worstRms
        worstRms=rateRms; [px,f]=pwelch(w(indices)-mean(w(indices)),[],[],[],1/Ts);
        use=f>.5 & f<80; fu=f(use); pu=px(use); [~,peak]=max(pu);
        spectrum=struct('Phase',p,'TailRateRms_deg_s',rad2deg(rateRms), ...
            'PeakFrequency_Hz',fu(peak),'Frequency_Hz',f,'Power',px);
    end
end
result.WorstHoldSpectrum=spectrum;
fprintf('Worst hold: phase %d, rate RMS %.3f deg/s, spectrum peak %.2f Hz\n', ...
    spectrum.Phase,spectrum.TailRateRms_deg_s,spectrum.PeakFrequency_Hz);
end
