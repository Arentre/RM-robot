function result = identify_gimbal_yaw_chirp(estimationInput,validationInput)
% 用独立的上扫和下扫试验拟合归一化指令模型。
result = gimbal_identify_integral(estimationInput,validationInput,"chirp",'Yaw');
end
