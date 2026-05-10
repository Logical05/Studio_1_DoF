% DC Motor Parameter Estimation
% Estimate:
% 1. Steady-state current (Iss)
% 2. Time constant (tau)
% 3. Current start time (t0)

clc;
clear;
close all;

%% Measured Data
% Startup region only

t_meas = [0 1 2 3 4 5];
I_meas = [0 0.66 0.99 0.99 0.99 0.99];
V = 6;

%% First-order delayed model
%
% I(t) = 0                          for t < t0
%
% I(t) = Iss*(1-exp(-(t-t0)/tau))  for t >= t0
%

model = @(p,t) ...
    (t >= p(3)) .* ...
    (p(1) .* (1 - exp(-(t - p(3))./p(2))));

% Parameters:
% p(1) = Iss
% p(2) = tau
% p(3) = t0

%% Initial guesses
Iss0 = 1.0;
tau0 = 0.5;
t00  = 0.2;

p0 = [Iss0 tau0 t00];

%% Lower and upper bounds
lb = [0 0 0];
ub = [5 10 1];

%% Parameter estimation
opts = optimoptions('lsqcurvefit','Display','off');

p_est = lsqcurvefit(model, ...
                    p0, ...
                    t_meas, ...
                    I_meas, ...
                    lb, ...
                    ub, ...
                    opts);

%% Estimated parameters
Iss_est = p_est(1);
tau_est = p_est(2);
t0_est  = p_est(3);
R = V / Iss_est; 
L = tau_est * R;

fprintf('\nEstimated Parameters\n');
fprintf('----------------------\n');
fprintf('Iss  = %.10f A\n', Iss_est);
fprintf('tau  = %.10f s\n', tau_est);
fprintf('t0   = %.10f s\n', t0_est);
fprintf('R   = %.10f Ohm\n', R);
fprintf('L   = %.10f H\n', L);

%% Generate fitted curve
t_fit = linspace(0,5,1000);
I_fit = model(p_est,t_fit);

%% Plot
figure('Color','w');

% Measured points
plot(t_meas, I_meas, 'bo', ...
    'MarkerSize',8, ...
    'MarkerFaceColor','b');

hold on;

% Estimated model
plot(t_fit, I_fit, 'r', ...
    'LineWidth',3);

% Mark estimated t0
xline(t0_est, '--k', ...
    sprintf('t0 = %.3f s', t0_est), ...
    'LineWidth',2);

%% Labels
xlabel('Time (s)');
ylabel('Motor Current (A)');

title('DC Motor Parameter Estimation with Unknown Start Time');

legend('Measured Data', ...
       'Estimated Model', ...
       'Estimated t0', ...
       'Location','southeast');

grid on;
box on;

%% Show equation
eqn = sprintf(['I(t)=I_{ss}(1-e^{-(t-t_0)/\\tau})\n' ...
               '\\tau=%.3f s,  t_0=%.3f s'], ...
               tau_est, t0_est);

text(2.2,0.3,eqn, ...
    'Interpreter','tex', ...
    'FontSize',12);