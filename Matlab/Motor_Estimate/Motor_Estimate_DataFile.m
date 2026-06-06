clc;
clear;
close all;

%% Configuration
Ts = 0.001;

files = {
    '5V-Chirp.csv'
    '5V-Step.csv'
    '5V-Stair.csv'
    '5V-Sine.csv'
    '5V-Ramp.csv'
    '10V-Chirp.csv'
    '10V-Step.csv'
    '10V-Stair.csv'
    '10V-Sine.csv'
    '10V-Ramp.csv'
};

%% Load All Experiments
expData = struct();

for k = 1:numel(files)

    filename = fullfile('data', files{k});

    T = readtable(filename);

    % Extract
    t_raw = T.time;
    u_raw = T.motor_voltage;
    y_raw = T.qei_omega;

    % Remove duplicate timestamps
    [t_raw, idx] = unique(t_raw);

    u_raw = u_raw(idx);
    y_raw = y_raw(idx);

    % Uniform time base
    t = (t_raw(1):Ts:t_raw(end))';

    % Resample
    u = interp1(t_raw,u_raw,t,'linear');
    y = interp1(t_raw,y_raw,t,'linear');

    % Create valid field name
    name = erase(files{k},'.csv');
    name = matlab.lang.makeValidName(name);

    % Store
    expData.(name).time = t;
    expData.(name).input = u;
    expData.(name).output = y;

    expData.(name).input_ts = timeseries(u,t);
    expData.(name).output_ts = timeseries(y,t);

end

names = fieldnames(expData);

for k = 1:numel(names)

    assignin('base', ...
        ['input_' names{k}], ...
        expData.(names{k}).input_ts);

    assignin('base', ...
        ['output_' names{k}], ...
        expData.(names{k}).output_ts);

end

%% Motor Parameter
motor_R = 9.047391929;     % [ohm]
motor_L = 0.001741906;     % [H]
motor_Kt = 2.594477849336442;    % [N * m/A]
motor_n = 0.794535644305898;
motor_B = 1.036743055651295;    % [N * m * S/rad]
motor_J = 0.092718896322627;    % [kg * m^2]

