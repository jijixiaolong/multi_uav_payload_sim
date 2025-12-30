% CHECK_CIRCLE_INIT - 检查圆形轨迹初始条件
clear; clc;

script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
addpath(project_dir);

[pd0, dpd0, d2pd0, ~, ~] = trajectory(0);
fprintf('圆形轨迹 t=0 时刻:\n');
fprintf('位置 pd = [%.4f, %.4f, %.4f]\n', pd0);
fprintf('速度 dpd = [%.4f, %.4f, %.4f]\n', dpd0);
fprintf('加速度 d2pd = [%.4f, %.4f, %.4f]\n', d2pd0);
fprintf('速度模: %.4f m/s\n', norm(dpd0));
fprintf('加速度模: %.4f m/s²\n\n', norm(d2pd0));

% 检查几个时间点
t_test = [0, 5, 10, 20];
fprintf('轨迹随时间变化:\n');
for i = 1:length(t_test)
    [pd, dpd, d2pd, ~, ~] = trajectory(t_test(i));
    fprintf('t=%.1f: pos=[%.3f,%.3f,%.3f], vel=[%.3f,%.3f,%.3f], acc=[%.3f,%.3f,%.3f]\n', ...
        t_test(i), pd, dpd, d2pd);
end

quit;
