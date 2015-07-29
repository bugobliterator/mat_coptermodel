load('jon_log_raw.mat');
start_sample_num = 5000;
end_sample_num = 10000;

%uniformize gyro data
start_time = IMU.data(start_sample_num,2);
end_time = IMU.data(end_sample_num,2) - start_time;
start_time = 0;
timestamp(:,1) = start_time:2.5:end_time;
IMU_time = IMU.data(start_sample_num:end_sample_num,2) - IMU.data(start_sample_num,2);
Gyro(:,1) = interp1(IMU_time,IMU.data(start_sample_num:end_sample_num,3),timestamp, 'spline');
Gyro(:,2) = interp1(IMU_time,IMU.data(start_sample_num:end_sample_num,4),timestamp, 'spline');
Gyro(:,3) = interp1(IMU_time,IMU.data(start_sample_num:end_sample_num,5),timestamp, 'spline');

%uniformize RC data
RCOU_time = RCOU.data(start_sample_num:end_sample_num,2) - IMU.data(start_sample_num,2);
RC(:,1) = interp1(RCOU_time,RCOU.data(start_sample_num:end_sample_num,3),timestamp, 'spline');
RC(:,2) = interp1(RCOU_time,RCOU.data(start_sample_num:end_sample_num,4),timestamp, 'spline');
RC(:,3) = interp1(RCOU_time,RCOU.data(start_sample_num:end_sample_num,5),timestamp, 'spline');
RC(:,4) = interp1(RCOU_time,RCOU.data(start_sample_num:end_sample_num,6),timestamp, 'spline');

%plot everything
figure
subplot(1,3,1)
plot(IMU_time,IMU.data(start_sample_num:end_sample_num,3),'o',timestamp,Gyro(:,1),':.');
legend('Gyro_x')
subplot(1,3,2)
plot(IMU_time,IMU.data(start_sample_num:end_sample_num,4),'o',timestamp,Gyro(:,2),':.');
legend('Gyro_y')
subplot(1,3,3)
plot(IMU_time,IMU.data(start_sample_num:end_sample_num,5),'o',timestamp,Gyro(:,3),':.');
legend('Gyro_z')

figure
subplot(2,2,1)
plot(RCOU_time,RCOU.data(start_sample_num:end_sample_num,3),'o',timestamp,RC(:,1),':.');
legend('RC1')
subplot(2,2,2)
plot(RCOU_time,RCOU.data(start_sample_num:end_sample_num,4),'o',timestamp,RC(:,2),':.');
legend('RC2')
subplot(2,2,3)
plot(RCOU_time,RCOU.data(start_sample_num:end_sample_num,5),'o',timestamp,RC(:,3),':.');
legend('RC3')
subplot(2,2,4)
plot(RCOU_time,RCOU.data(start_sample_num:end_sample_num,6),'o',timestamp,RC(:,4),':.');
legend('RC4')
