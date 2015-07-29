function res = error_func(Ki, Ktr, Im, Kd, x_gain, y_gain, z_gain, yaw_misalignment, system_lag, cg_x, cg_y, base_thr,thr_data, diff_imu)
    dt = 0.0025;
    residual = 0;
    for i = drange(1:size(diff_imu))
            [dwx,dwy,dwz] = model(i, Ki, Ktr, Im, Kd, x_gain, y_gain, z_gain, yaw_misalignment, system_lag, cg_x, cg_y, base_thr,thr_data, dt);
            res_dwx = dwx - diff_imu(i,1);
            res_dwy = dwy - diff_imu(i,2);
            res_dwz = dwz - diff_imu(i,3);
            residual = residual + res_dwx^2 + res_dwy^2 + res_dwz^2;
    end
    res = residual
    [Ki, Ktr, Im, Kd, x_gain, y_gain, z_gain, yaw_misalignment, system_lag, cg_x, cg_y,base_thr]
end