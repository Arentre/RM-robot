function result = identify_gimbal_pitch(estimationInput,validationInput)
% 在一个姿态附近辨识 Pitch 增益，用常量偏置近似重力。
% 独立试验应使用相同机械配置和起始姿态。
if nargin < 2, validationInput = []; end
result = gimbal_identify_integral(estimationInput,validationInput, ...
    ["positive_step","return_step"],'Pitch');
end
