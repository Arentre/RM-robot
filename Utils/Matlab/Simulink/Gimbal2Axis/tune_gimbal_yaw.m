function summary = tune_gimbal_yaw(port,b0,wcCandidates,step_deg,outputLimit)
if nargin < 3, wcCandidates = [2 4 6 8]; end
if nargin < 4, step_deg = 5; end
if nargin < 5, outputLimit = 0.40; end
summary = gimbal_tune_ladrc(port,'Y',b0,wcCandidates,step_deg,outputLimit);
end
