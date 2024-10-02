test_motor_kp = 1000;
test_motor_ki = 10;
test_motor_kd = 30;

max_torque_limit = 100;   % Nm
min_torque_limit = -100;  % Nm

B = [0; 0; 1];

filter_coeff = 1000;