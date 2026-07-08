clear; close all; clc;

%% Project paths

script_path = mfilename('fullpath');
if isempty(script_path)
    project_dir = pwd;
else
    project_dir = fileparts(script_path);
end

addpath(project_dir);

output_dir = fullfile(project_dir, 'Graficos');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

%% Simulation model

simulation_name = "TCC";
matlab_version = 2024;
model_name = sprintf('sim_%s_%d', simulation_name, matlab_version);
model_file = fullfile(project_dir, [model_name '.slx']);

if ~isfile(model_file)
    error('Model file not found: %s', model_file);
end

load_system(model_file);

%% Time and trajectory setup

N_cycles = 50;
Tspan = 10;
shift_time = Tspan / 2;
Tsim = N_cycles * Tspan;
end_learning_time = 5 * Tspan;

Ts = 0.01;
Ts_sim = Ts / 10;

t_pick = 0:Ts:Tspan/2-Ts;
t_return = 0:Ts:Tspan/2-Ts;

[q0_ida, qd0_ida, qdd0_ida] = trajetoria(0, 120, t_pick);
[q1_ida, qd1_ida, qdd1_ida] = trajetoria(0, -90, t_pick);
[q2_ida, qd2_ida, qdd2_ida] = trajetoria(0, 90, t_pick);

[q0_volta, qd0_volta, qdd0_volta] = trajetoria(120, -120, t_return);
[q1_volta, qd1_volta, qdd1_volta] = trajetoria(-90, 90, t_return);
[q2_volta, qd2_volta, qdd2_volta] = trajetoria(90, -90, t_return);

%% Controller and disturbance setup

gama1 = 100;
gama2 = 1;
gama3 = 1;

lambda_ILC = [0.99, 0.99, 0.99];
Lim_Torque_motor = 1.4; % Stall torque [N.m] for the MG995R servo.
SATURATION_ON = 1;

noise_power = [
    2.62e-4, 7.62e-9, ...
    2.62e-4, 7.62e-9, ...
    2.62e-4, 7.62e-9];

dist = 0.1 * Lim_Torque_motor * [1, 1, 1];
t0_dist = end_learning_time + 2;
tf_dist = t0_dist + 2;

initial_error_min = 1.0;
initial_error_max = 1.5;
error_initial_condition = initial_error_min ...
    + (initial_error_max - initial_error_min) * rand(1, 3);
error_difference = [
    error_initial_condition(1), 0, ...
    error_initial_condition(2), 0, ...
    error_initial_condition(3), 0];

m_payload = 0.125; % Payload mass variation [kg].

gain_sets = [
    30, 2.5, 30, 2.5, 30, 2.5
    30, 0.5, 30, 0.5, 30, 0.5
];

scenario_names = {
    'Ideal'
    'Ruido branco'
    'Perturbacao degrau'
    'Payload'
    'Erro de condicao inicial'
};

num_scenarios = numel(scenario_names);
control_methods = {
    struct('label', 'PD classico', 'ILC_ON', 0, 'ILC_ADAPT_ON', 0, 'prefix', 'PD')
    struct('label', 'PD + ILC', 'ILC_ON', 1, 'ILC_ADAPT_ON', 1, 'prefix', 'ILC')
};

%% Automation loop

for method_index = 1:numel(control_methods)
    method = control_methods{method_index};
    ILC_ON = method.ILC_ON;
    ILC_ADAPT_ON = method.ILC_ADAPT_ON;

    fprintf('\n=== %s ===\n', method.label);

    for gain_version = 1:size(gain_sets, 1)
        gains = gain_sets(gain_version, :);

        Kp0 = gains(1);
        Kv0 = gains(2);
        Kp1 = gains(3);
        Kv1 = gains(4);
        Kp2 = gains(5);
        Kv2 = gains(6);

        for scenario = 1:num_scenarios
            fprintf( ...
                '\n=== Scenario %d: %s | Gains %d ===\n', ...
                scenario, ...
                scenario_names{scenario}, ...
                gain_version);

            NOISE_ON = 0;
            DISTURBANCE_ON = 0;
            FREEZE_ON = 0;
            IC_ERROR_ON = 0;
            PAYLOAD_ON = 0;

            switch scenario
                case 2
                    NOISE_ON = 1;
                case 3
                    DISTURBANCE_ON = 1;
                case 4
                    PAYLOAD_ON = 1;
                case 5
                    IC_ERROR_ON = 1;
            end

            sim(model_name);

            t_sim = juntas.Time;
            e_rho = e_pos0;
            e_the1 = e_pos1;
            e_the2 = e_pos2;

            u_rho = u0;
            u_the1 = u1;
            u_the2 = u2;

            error_signals = {e_rho, e_the1, e_the2};
            torque_signals = {u_rho, u_the1, u_the2};

            metric_Energy = calculateMetricBySignal( ...
                torque_signals, ...
                @(signal) AnalyseEnergy(signal, Tsim, Tspan, Ts_sim));
            metric_MEPI = calculateMetricBySignal( ...
                error_signals, ...
                @(signal) MaxErrorPerIteration(signal, Tsim, Tspan, Ts_sim));
            metric_ISE = calculateMetricBySignal( ...
                error_signals, ...
                @(signal) ISE(signal, Tsim, Tspan, Ts_sim));
            metric_RMSE = calculateMetricBySignal( ...
                error_signals, ...
                @(signal) RMSE(signal, Tsim, Tspan, Ts_sim));

            output_file = fullfile( ...
                output_dir, ...
                sprintf( ...
                    '%s_Dados_Cenario_%d_Gains_%d.mat', ...
                    method.prefix, ...
                    scenario, ...
                    gain_version));

            save( ...
                output_file, ...
                't_sim', ...
                'juntas', ...
                'u_rho', ...
                'u_the1', ...
                'u_the2', ...
                'e_rho', ...
                'e_the1', ...
                'e_the2', ...
                'metric_MEPI', ...
                'metric_ISE', ...
                'metric_Energy', ...
                'metric_RMSE', ...
                'error_initialCondition', ...
                'm_payload', ...
                'gains', ...
                'disturbances', ...
                'noises', ...
                'm3');

            fprintf('Saved: %s\n', output_file);
        end
    end
end

disp('=== All simulations finished and saved. ===');

function metric = calculateMetricBySignal(signals, metric_function)
    metric = [];

    for signal_index = 1:numel(signals)
        metric = [metric, metric_function(signals{signal_index})]; %#ok<AGROW>
    end
end

function result = AnalyseEnergy(torque, Tsim, Tspan, Ts_sim)
    torque_data = torque.Data;
    t_plot = torque.Time;
    result = [];

    for n = 1:(Tsim/Tspan)
        i_start = (n - 1) * Tspan / Ts_sim + 1;
        i_end = n * Tspan / Ts_sim;

        Energy_calc = trapz( ...
            t_plot(i_start:i_end), ...
            torque_data(i_start:i_end).^2);
        result = [result; Energy_calc]; %#ok<AGROW>
    end
end

function result = MaxErrorPerIteration(error_ts, Tsim, Tspan, Ts_sim)
    e_pos = error_ts.Data;
    result = [];

    for n = 1:(Tsim/Tspan)
        i_start = (n - 1) * Tspan / Ts_sim + 1;
        i_end = n * Tspan / Ts_sim;

        MEPI_calc = max(abs(e_pos(i_start:i_end)));
        result = [result; MEPI_calc]; %#ok<AGROW>
    end
end

function result = ISE(error_ts, Tsim, Tspan, Ts_sim)
    e_pos = error_ts.Data;
    t_plot = error_ts.Time;
    result = [];

    for n = 1:(Tsim/Tspan)
        i_start = (n - 1) * Tspan / Ts_sim + 1;
        i_end = n * Tspan / Ts_sim;

        ISE_calc = trapz(t_plot(i_start:i_end), e_pos(i_start:i_end).^2);
        result = [result; ISE_calc]; %#ok<AGROW>
    end
end

function result = RMSE(error_ts, Tsim, Tspan, Ts_sim)
    e_pos = error_ts.Data;
    result = [];

    for n = 1:(Tsim/Tspan)
        i_start = (n - 1) * Tspan / Ts_sim + 1;
        i_end = n * Tspan / Ts_sim;

        RMSE_calc = sqrt(mean(e_pos(i_start:i_end).^2));
        result = [result; RMSE_calc]; %#ok<AGROW>
    end
end
