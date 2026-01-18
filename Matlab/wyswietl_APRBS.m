clear; clc; close all;

%% 1. Ładowanie danych
% Sprawdzamy czy plik z danymi istnieje
data_file = 'dane_wymuszenia.mat';

if ~exist(data_file, 'file')
    error('Nie znaleziono pliku %s. Najpierw uruchom generowanie_APRBS.m!', data_file);
end

load(data_file);
fprintf('Wczytano dane z pliku: %s\n', data_file);

%% 2. Sprawdzenie zmiennych
if ~exist('pwm_center', 'var')
    % Jeśli pwm_center nie zostało zapisane, przyjmujemy domyślne 1500
    pwm_center = 1500;
end

%% 3. Wizualizacja (Sprawdź czy to wygląda bezpiecznie!)
figure;
plot(t, u_pwm, 'LineWidth', 1.5);
grid on;
xlabel('Czas [s]');
ylabel('Szerokość impulsu PWM [us]');
title('Sygnał wymuszający dla Serwa (Input)');
yline(pwm_center, 'r--', 'Poziom');
ylim([1000 2000]); % Pokazujemy szerszy zakres dla kontekstu
