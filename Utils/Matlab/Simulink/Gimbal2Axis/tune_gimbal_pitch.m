function summary = tune_gimbal_pitch(port,b0,wcCandidates,step_deg,outputLimit)
% 先确认机械限位，从 Pitch 行程中间开始试验。
if nargin < 3, wcCandidates = [1 2 3 4]; end
if nargin < 4, step_deg = 2; end
if nargin < 5, outputLimit = 0.35; end
summary = gimbal_tune_ladrc(port,'P',b0,wcCandidates,step_deg,outputLimit);
end
