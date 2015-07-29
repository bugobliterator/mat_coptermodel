init_params = [7.72211506170580,3.61237849272872,0.875798727754292,0.0601088298664231,0.0289246437419706,0.0214258859125666,0.0175623565007145,-0.0233457898974756,0.0266614159189289,0.487800029016314,0.518964277197835,1117.0];
load('jon_log.mat');
imu_data = Gyro;
diff_imu = zeros(1,size(imu_data,2));
dt = 0.0025;
for i = drange(2:size(Gyro,1)-1)
    diff_imu = cat(1,diff_imu,(((imu_data(i+1,:) - imu_data(i,:)) + (imu_data(i,:) - imu_data(i-1,:)))/(2*dt)));
end
thr_data = RC;
lb = [0.001 0.001 0.1 0.001 0.01 0.01 0.0001 -0.1 0.0  0.0 0.0 1000];
ub = [100.0 100.0 100 100.0 10.0 10.0 10.0    0.1 0.05 1.0 1.0 1150];
func = @(param)error_func(param(1), param(2),param(3),param(4),param(5),param(6),param(7),param(8),param(9),param(10),param(11),param(12),thr_data, diff_imu); 
%p = gamultiobj(func,12,[],[],[],[],lb,ub);
options = optimoptions(@fsolve,...
    'Display','iter','Algorithm','levenberg-marquardt');
p = fsolve(func, init_params, options)