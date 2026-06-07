clc;
clear;
close all;

Ts = 0.001;

input_folder  = "data";
output_folder = "data_resampled";

if ~exist(output_folder,'dir')
    mkdir(output_folder);
end

files = {
    'S-Curve.csv'
    'Trapezoid.csv'
};

for k = 1:numel(files)

    filename = fullfile(input_folder, files{k});

    fprintf('Processing %s\n', files{k});
    
    T = readtable(filename, ...
        'VariableNamingRule','preserve');

    %% Read columns
    t_raw = T.time;
    
    qei_omega = T.("qei.omega");
    qei_omega_filtered = T.("qei.omega_filtered");
    qei_alpha_estimate = T.("qei.alpha_estimate");
    
    reference_position = T.("reference.position");
    reference_velocity = T.("reference.velocity");
    reference_acceleration = T.("reference.acceleration");
    
    %% Remove duplicate timestamps
    [t_raw, idx] = unique(t_raw);
    
    qei_omega = qei_omega(idx);
    qei_omega_filtered = qei_omega_filtered(idx);
    qei_alpha_estimate = qei_alpha_estimate(idx);
    
    reference_position = reference_position(idx);
    reference_velocity = reference_velocity(idx);
    reference_acceleration = reference_acceleration(idx);
    
    %% Uniform time vector
    t_new = (t_raw(1):Ts:t_raw(end))';
    
    %% Resample
    qei_omega_new = interp1( ...
        t_raw, qei_omega, ...
        t_new, 'linear');
    
    qei_omega_filtered_new = interp1( ...
        t_raw, qei_omega_filtered, ...
        t_new, 'linear');
    
    qei_alpha_estimate_new = interp1( ...
        t_raw, qei_alpha_estimate, ...
        t_new, 'linear');
    
    reference_position_new = interp1( ...
        t_raw, reference_position, ...
        t_new, 'linear');
    
    reference_velocity_new = interp1( ...
        t_raw, reference_velocity, ...
        t_new, 'linear');
    
    reference_acceleration_new = interp1( ...
        t_raw, reference_acceleration, ...
        t_new, 'linear');
    
    %% Create output table
    Tout = table( ...
        t_new, ...
        qei_omega_new, ...
        qei_omega_filtered_new, ...
        qei_alpha_estimate_new, ...
        reference_position_new, ...
        reference_velocity_new, ...
        reference_acceleration_new, ...
        'VariableNames', { ...
            'time', ...
            'qei_omega', ...
            'qei_omega_filtered', ...
            'qei_alpha_estimate', ...
            'reference_position', ...
            'reference_velocity', ...
            'reference_acceleration'});

    [~, name, ~] = fileparts(files{k});

    output_file = fullfile( ...
        output_folder, ...
        strcat(name, "_resampled.csv"));

    writetable(Tout, output_file);

end

fprintf('\nFinished.\n');