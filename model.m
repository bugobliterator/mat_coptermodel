function [dwx,dwy,dwz] = model(i, Ki, Ktr, Im, Kd, x_gain, y_gain, z_gain, yaw_misalignment, system_lag, cg_x, cg_y, base_thr, thr_data, dt)
    motor_params = [Ki, Ktr, Im, Kd];
    cos_yaw_misalignment = cos(yaw_misalignment);
    sin_yaw_misalignment = sin(yaw_misalignment);
    alpha = dt/(dt+system_lag);
    thr = fetch_thr(i, base_thr, thr_data);     %fetch throttle value for each motor from logs
    [ang_vel, torque] = update_motors(thr,motor_params,dt);      %update motor states
    persistent omega_x
    if isempty(omega_x)
        omega_x =0;
    end
    persistent omega_y
    if isempty(omega_y)
        omega_y =0;
    end
    persistent omega_z
    if isempty(omega_z)
        omega_z =0;
    end
    %calc angular velocities in each axis
    omega_dot_x = ((cg_x*(ang_vel(3)^2 + ang_vel(2)^2)) - ((1-cg_x)*(ang_vel(1)^2 + ang_vel(4)^2))) * x_gain;
    omega_dot_y = ((cg_y*(ang_vel(3)^2 + ang_vel(1)^2)) - ((1-cg_y)*(ang_vel(2)^2 + ang_vel(4)^2))) * y_gain;
    omega_dot_z = (torque(1) + torque(2) - torque(3) - torque(4)) * z_gain;

    %apply histerisis to angular acceleration in each axis
    omega_dot_x = filter(alpha, (omega_dot_x * cos_yaw_misalignment) - (omega_dot_y * sin_yaw_misalignment),1);
    omega_dot_y = filter(alpha, (omega_dot_x * sin_yaw_misalignment) + (omega_dot_y * cos_yaw_misalignment),2);
    omega_dot_z = filter(alpha, omega_dot_z,3);

    %get angular velocities using previous angular velocities
    omega_x = omega_x + omega_dot_x * dt;
    omega_y = omega_y + omega_dot_y * dt;
    omega_z = omega_z + omega_dot_z * dt;

    dwx = omega_x;
    dwy = omega_y;
    dwz = omega_z;
end

function [ang_vel, torque] = update_motors(thr, motor_params, dt)
    Ki = motor_params(1); % back-emf constant of motor
    Ktr = motor_params(2); % torque constant of motor/resistance of motor
    Im = motor_params(3); % moment of inertia of prop-motor system
    Kd = motor_params(4); % 0.5*p*Cd*A
    t_motor=[0,0,0,0];
    persistent omega
    if isempty(omega)
        omega = zeros(1,4);
    end
    for k = drange(1:4)
        V_motor = max(0,thr(k) - Ki*omega(k));
        t_motor(k) = Ktr * V_motor;
        omega_dot = (t_motor(k) - Kd * omega(k)^2) / Im;
        omega(k) = omega(k) + omega_dot*dt;
    end
    ang_vel = omega;
    torque = t_motor;
end
function f = filter(alpha,y,k)
    persistent filtered;
    if isempty(filtered)
        filtered=zeros(1,3);
    end
    filtered(k) = filtered(k) + (y-filtered(k))*alpha;
    f = filtered(k);
end
function thr = fetch_thr(i, base_thr, thr_data)
    thr(1) = thr_data(i,1) - base_thr;
    thr(2) = thr_data(i,2) - base_thr;
    thr(3) = thr_data(i,3) - base_thr;
    thr(4) = thr_data(i,4) - base_thr;
end