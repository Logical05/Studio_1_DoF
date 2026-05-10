clear

motor_R = 9.047391929;
motor_L = 0.001741906;
motor_K = 2.2321;
motor_B = 0.78891;
motor_J = 0.029842;

% denominator:
% LJs^2 + (RJ + BL)s + (BR + K^2)

den = [
    motor_L * motor_J
    motor_R * motor_J + motor_B * motor_L
    motor_B * motor_R + motor_K^2
];

% Velocity TF
plant_velocity = tf(motor_K, den);

% Position TF
plant_theta = plant_velocity * tf(1,[1 0]);

controlSystemDesigner(plant_theta);