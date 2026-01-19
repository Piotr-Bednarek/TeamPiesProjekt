clear all; close all; clc;

data = readtable('odp-na-skok.csv');

% Wyświetl podstawowe informacje
figure('Position', [100, 100, 1200, 800]);

start_idx = 150;
data = data(start_idx:end, :);

plot(data.time, data.filtered, 'LineWidth', 2, 'DisplayName', 'filtered'); hold on;
plot(data.time, data.control, 'LineWidth', 1.5, 'DisplayName', 'control');

legend;
grid on;
hold off;
