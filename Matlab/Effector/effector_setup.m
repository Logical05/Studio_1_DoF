clc;
clear;
close all;

%% ================= PARAMETERS =================
g  = 9.806;             % Gravity (m/s^2)
L1 = 0.278;             % Length of Arm 1 (m)
L2 = 0.300;             % Length of Arm 2 (m)
m1 = 0.300;             % Mass of Arm 1 (kg)
m2 = 0.075;             % Mass of Arm 2 (kg)
l1 = 0.150;             % Center of mass of Arm 1 (m)
l2 = 0.148;             % Center of mass of Arm 2 (m)
J1 = 2.48 * 10^-2;      % Inertia of Arm 1 (kg·m^2)
J2 = 3.86 * 10^-3;      % Inertia of Arm 2 (kg·m^2)
b1 = 1.00 * 10^-4;      % Damping coefficient Arm 1 (N·m·s)
b2 = 2.80 * 10^-4;      % Damping coefficient Arm 2 (N·m·s)

%% ================= SHORTCUT TERMS =================
J_1 = J1 + (m1 * l1^2);
J_2 = J2 + (m2 * l2^2);
J_0 = J_1 + (m2 * L1^2);
alpha = (J_0 * J_2) - (m2 * L1 * l2)^2;

%% ================= STATE SPACE =================
A = [
    0   0   1   0;
    0   0   0   1;
    0    (g * m2^2 * l2^2 * L1)/alpha     (-b1 * J_2)/alpha               -(-b2 * m2 * l2 * L1)/alpha;
    0   -(g * m2 * l2 * J_0)/alpha       -(-b1 * m2 * l2 * L1)/alpha       (-b2 * J_0)/alpha;
];

B = [
    0   0;
    0   0;
    J_2/alpha               -(m2 * L1 * l2)/alpha;
    -(m2 * L1 * l2)/alpha    J_0/alpha;
];

C = eye(4);
D = zeros(4, 2);
Initial = zeros(4, 1);