
% Motor Parameters
motor_R = 9.047391929;      % Ohm
motor_L = 0.001741906;      % H
motor_K = 2.2321;           % Nm/A
motor_B = 0.78891;          % Nms/rad
motor_J = 0.029842;         % kgm^2

s = tf('s');

% Velocity Plant
den = (motor_L*s + motor_R) * ...
      (motor_J*s + motor_B) + ...
      motor_K^2;

Gv = motor_K / den;

% Position Plant
Gp = Gv / s;

% controlSystemDesigner(Gv)

Tv = feedback(C * Gv, 1);

s = tf('s');

Gp_outer = Tv / s;

controlSystemDesigner(Gp_outer)