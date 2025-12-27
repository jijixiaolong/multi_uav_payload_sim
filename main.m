% Main Simulation Script
clear; clc; close all;

% Add paths
current_dir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(current_dir, 'dynamics')));
addpath(genpath(fullfile(current_dir, 'control')));
addpath(genpath(fullfile(current_dir, 'math')));
addpath(genpath(fullfile(current_dir, 'utils')));
addpath(genpath(fullfile(current_dir, 'plot')));

% Load parameters
p = params();

% Initialize state
x0 = init_state(p.n);

% Time span
tspan = [0 20];

% ODE Solver
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
[t, x] = ode45(@(t, x) system_dynamics(t, x, p), tspan, x0, options);

% Plot results
plot_payload(t, x, p);
plot_cable(t, x, p);
plot_disturb(t, x, p);
plot_input(t, x, p);

disp('Simulation complete.');
