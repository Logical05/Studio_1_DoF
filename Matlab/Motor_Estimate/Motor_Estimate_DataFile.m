% Load Data
data_12V = readtable("data\12V-2.csv");
data_24V = readtable("data\24V-2.csv");

% Extract collected data
idx_12V = height(data_12V);
idx_24V = height(data_24V);

% Time vector
dt = 0.001;

Time_12V = (0:idx_12V-1)' * dt;
Omega_12V = data_12V.QEIdata_omega;
Theta_12V = data_12V.QEIdata_theta;
voltage_12V = data_12V.Voltage(1);

Time_24V = (0:idx_24V-1)' * dt;
Omega_24V = data_24V.QEIdata_omega;
Theta_24V = data_24V.QEIdata_theta;
voltage_24V = data_24V.Voltage(1);


% Motor Parameter
motor_R = 0.600390664094146;      % [ohm]
motor_L = 0.00140182548257971;     % [H]
motor_K = 4.36661891100601;     % [V * S/rad]
motor_B = 9.37860305468294;     % [N * m * S/rad]
motor_J = 0.311429444326669;        % [kg * m^2]