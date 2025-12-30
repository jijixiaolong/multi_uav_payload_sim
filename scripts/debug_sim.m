% Debug script to run system_dynamics once
clc; clear; close all;

% Add paths
script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
addpath(genpath(project_dir));

% Load parameters
p = params();

% Initialize state
x0 = init_state(p);

% Run one step
t = 0;
try
    dx = system_dynamics(t, x0, p);
    disp('System dynamics run successfully!');
    disp(['dx size: ', num2str(size(dx))]);
catch ME
    disp('Error in system_dynamics:');
    disp(ME.message);
    disp('Stack trace:');
    for i = 1:length(ME.stack)
        disp(['File: ', ME.stack(i).file]);
        disp(['Name: ', ME.stack(i).name]);
        disp(['Line: ', num2str(ME.stack(i).line)]);
    end
end
