params = [7.51810770e+00, 3.58577766e+00,   8.64518317e-01,5.87832121e-02,   2.84742353e-02,   2.13692233e-02,1.74909129e-02,  -2.35811974e-02,   2.66885902e-02,4.86427532e-01,   5.37986428e-01,   990.0];
param_list = num2cell(params);
[Ki, Ktr, Im, Kd, x_gain, y_gain, z_gain, yaw_misalignment, system_lag, cg_x, cg_y, base_thr] = param_list{:};
load('jon_log.mat');
dt = 0.0025;
imu_data = Gyro;
diff_imu = zeros(1,size(imu_data,2));
for i = drange(2:size(Gyro,1)-1)
    diff_imu = cat(1,diff_imu,(((imu_data(i+1,:) - imu_data(i,:)) + (imu_data(i,:) - imu_data(i-1,:)))/(2*dt)));
end
thr_data = RC;
dw = [0,0,0];
for i = drange(1:size(RC,1))
        [dwx,dwy,dwz] = model(i, Ki, Ktr, Im, Kd, x_gain, y_gain, z_gain, yaw_misalignment, system_lag, cg_x, cg_y, base_thr,thr_data, dt);
        dw = cat(1,dw,[dwx,dwy,dwz]);
end
t = timestamp;
size(t)
size(dw)
size(diff_imu)
figure
subplot(3,1,1)
plot(t,detrend(dw(2:size(dw),1)));
legend('Model');
hold
plot(t(1:size(t)),imu_data(:,1));
legend('Actual');

subplot(3,1,2)
plot(t,detrend(dw(2:size(dw),2)));
legend('Model');
hold
plot(t(1:size(t)),imu_data(:,2));
legend('Actual');

subplot(3,1,3)
plot(t,detrend(dw(2:size(dw),3)));
legend('Model');
hold
plot(t(1:size(t)),imu_data(:,3));
legend('Actual');