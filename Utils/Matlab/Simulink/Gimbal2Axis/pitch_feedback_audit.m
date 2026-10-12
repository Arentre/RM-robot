function audit = pitch_feedback_audit(data)
% Compare independently measured motion; filtering is identical for all rates.
assert(license('test','Signal_Toolbox') && license('test','Statistics_Toolbox'), ...
    'Signal Processing and Statistics toolboxes are required.');
d=data(data.Phase>=1,:);
[t,i]=unique(d.McuTime_s,'stable'); d=d(i,:);
Ts=.005; tu=(t(1):Ts:t(end)).';
imu=interp1(t,deg2rad(d.PitchFeedback_deg),tu,'linear');
enc=interp1(t,unwrap(deg2rad(d.PitchEncoder_deg)),tu,'linear');
gyro=interp1(t,[d.GyroX_deg_s d.GyroY_deg_s d.GyroZ_deg_s],tu,'linear');
[b,a]=butter(3,3/(.5/Ts));
imuRate=rad2deg(gradient(filtfilt(b,a,imu),Ts));
encoderRate=rad2deg(gradient(filtfilt(b,a,enc),Ts));
gyroRate=filtfilt(b,a,gyro);
moving=abs(gyroRate(:,2))>3 & abs(gyroRate(:,2))<50;
moving(1:min(100,end))=false; moving(max(1,end-100):end)=false;
assert(nnz(moving)>30,'Insufficient motion for feedback audit.');
audit=struct();
audit.AxisCorrelations=corr(imuRate(moving),gyroRate(moving,:));
audit.ImuRateFromGyro=robustfit(gyroRate(moving,2),imuRate(moving));
audit.EncoderRateFromGyro=robustfit(gyroRate(moving,2),encoderRate(moving));
audit.ImuRateFromEncoder=robustfit(encoderRate(moving),imuRate(moving));
audit.ImuGyroRateRMSE=rms(imuRate(moving)-gyroRate(moving,2));
audit.EncoderGyroRateRMSE=rms(encoderRate(moving)-gyroRate(moving,2));
phase=interp1(t,d.Phase,tu,'previous');
audit.Spectra=table();
for k=unique(phase).'
    indices=find(phase==k);
    indices=indices(tu(indices)>=tu(indices(end))-1);
    if numel(indices)<80, continue; end
    rate=gyro(indices,2);
    [px,f]=pwelch(rate-mean(rate),hamming(min(128,numel(indices))),[],1024,1/Ts);
    band=f>.5 & f<70; fb=f(band); pb=px(band); [~,j]=max(pb);
    audit.Spectra=[audit.Spectra;table(k,rms(rate),fb(j), ...
        'VariableNames',{'Phase','TailRateRMS','PeakFrequency_Hz'})]; %#ok<AGROW>
end
fprintf('IMU derivative correlations with gyro [x y z]: %.3f %.3f %.3f\n',audit.AxisCorrelations);
fprintf('Matched 3Hz filtering slopes: IMU/gyroY %.3f encoder/gyroY %.3f IMU/encoder %.3f\n', ...
    audit.ImuRateFromGyro(2),audit.EncoderRateFromGyro(2),audit.ImuRateFromEncoder(2));
fprintf('Rate residual RMS: IMU-gyroY %.3f encoder-gyroY %.3f deg/s\n', ...
    audit.ImuGyroRateRMSE,audit.EncoderGyroRateRMSE);
disp(audit.Spectra);
end
