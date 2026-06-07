clc;
clear;
close all;

%% Configuration
Ts = 0.001; % 1 ms

input_folder  = "data";
output_folder = "data_resampled";

if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end

%% File List
files = {
    'Chirp_1.csv'
    'Chirp_1e+3.csv'
    'Chirp_1e-3.csv'
    'Chirp_1e-6.csv'
    'Chirp_1e-9.csv'
};


%% Process Files
for k = 1:numel(files)

    filename = fullfile(input_folder, files{k});

    fprintf('Processing %s\n', files{k});

    T = readtable(filename, ...
        'VariableNamingRule','preserve');

   %% Read columns
    t_raw = T.time;
    
    kf_x0 = T.("kf.x[0]");
    kf_x1 = T.("kf.x[1]");
    kf_x2 = T.("kf.x[2]");
    kf_x3 = T.("kf.x[3]");
    
    qei_alpha = T.("qei.alpha_estimate");
    qei_omega = T.("qei.omega");
    qei_omega_f = T.("qei.omega_filtered");
    qei_theta = T.("qei.theta");
    
    motor_voltage = T.motor_voltage;
    
    %% Remove duplicate timestamps
    [t_raw, idx] = unique(t_raw);
    
    kf_x0 = kf_x0(idx);
    kf_x1 = kf_x1(idx);
    kf_x2 = kf_x2(idx);
    kf_x3 = kf_x3(idx);
    
    qei_alpha = qei_alpha(idx);
    qei_omega = qei_omega(idx);
    qei_omega_f = qei_omega_f(idx);
    qei_theta = qei_theta(idx);
    
    motor_voltage = motor_voltage(idx);
    
    %% Uniform time vector
    t_new = (0:Ts:10)';
    
    %% Resample
    kf_x0_new = interp1(t_raw, kf_x0, t_new, 'linear', 'extrap');
    kf_x1_new = interp1(t_raw, kf_x1, t_new, 'linear', 'extrap');
    kf_x2_new = interp1(t_raw, kf_x2, t_new, 'linear', 'extrap');
    kf_x3_new = interp1(t_raw, kf_x3, t_new, 'linear', 'extrap');
    
    qei_alpha_new = interp1(t_raw, qei_alpha, t_new, 'linear', 'extrap');
    qei_omega_new = interp1(t_raw, qei_omega, t_new, 'linear', 'extrap');
    qei_omega_f_new = interp1(t_raw, qei_omega_f, t_new, 'linear', 'extrap');
    qei_theta_new = interp1(t_raw, qei_theta, t_new, 'linear', 'extrap');
    
    motor_voltage_new = interp1(t_raw, motor_voltage, t_new, 'linear', 'extrap');
    
    %% Create output table
    Tout = table( ...
        t_new, ...
        kf_x0_new, ...
        kf_x1_new, ...
        kf_x2_new, ...
        kf_x3_new, ...
        qei_alpha_new, ...
        qei_omega_new, ...
        qei_omega_f_new, ...
        qei_theta_new, ...
        motor_voltage_new, ...
        'VariableNames', { ...
            'time', ...
            'kf_x0', ...
            'kf_x1', ...
            'kf_x2', ...
            'kf_x3', ...
            'qei_alpha_estimate', ...
            'qei_omega', ...
            'qei_omega_filtered', ...
            'qei_theta', ...
            'motor_voltage'});

    [~, name, ~] = fileparts(files{k});

    output_file = fullfile( ...
        output_folder, ...
        strcat(name, "_resampled.csv"));
    
    writetable(Tout, output_file);

end

fprintf('\nFinished.\n');