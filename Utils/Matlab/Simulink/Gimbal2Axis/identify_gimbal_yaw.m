function result = identify_gimbal_yaw(estimationInput,validationInput)
% 使用独立的双向台架试验估计并验证增益。
% 只有一组采集时仅输出诊断系数。
if nargin < 2, validationInput = []; end
result = gimbal_identify_integral(estimationInput,validationInput, ...
    ["positive_step","return_step"],'Yaw');
end
