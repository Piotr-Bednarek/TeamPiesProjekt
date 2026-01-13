%% Analiza oscylacji - Metoda Zieglera-Nicholsa
% Wczytanie danych z pliku CSV i obliczenie okresu oscylacji Tu

clear; clc; close all;

%% Wczytanie danych
data = readtable('../przebieg.csv');

time = data.time;
distance = data.distance;
filtered = data.filtered;
setpoint = data.setpoint;
error_sig = data.error;

%% Wyświetlenie przebiegu
figure('Name', 'Przebieg oscylacji', 'Position', [100 100 1200 600]);

subplot(2,1,1);
plot(time, distance, 'g:', 'LineWidth', 1, 'DisplayName', 'Dystans RAW');
hold on;
plot(time, filtered, 'b-', 'LineWidth', 2, 'DisplayName', 'Dystans filtrowany');
plot(time, setpoint, 'r--', 'LineWidth', 2, 'DisplayName', 'Setpoint');
xlabel('Czas [s]');
ylabel('Pozycja [mm]');
title('Przebieg oscylacji systemu Ball on Beam');
legend('Location', 'best');
grid on;
ylim([0 260]);

subplot(2,1,2);
plot(time, error_sig, 'r-', 'LineWidth', 1.5);
xlabel('Czas [s]');
ylabel('Uchyb [mm]');
title('Uchyb regulacji');
grid on;
yline(0, 'k--', 'LineWidth', 1);

%% Detekcja przejść przez zero uchybu (metoda przecięć)
% Szukamy momentów gdy uchyb zmienia znak z + na - (przejście przez setpoint od dołu)

zero_crossings = [];
for i = 2:length(error_sig)
    % Przejście z dodatniego na ujemny (piłka przechodzi przez setpoint jadąc w górę)
    if error_sig(i-1) > 0 && error_sig(i) <= 0
        % Interpolacja liniowa dla dokładniejszego wyniku
        t_cross = time(i-1) + (0 - error_sig(i-1)) / (error_sig(i) - error_sig(i-1)) * (time(i) - time(i-1));
        zero_crossings = [zero_crossings; t_cross];
    end
end

fprintf('=== ANALIZA OKRESU OSCYLACJI ===\n\n');
fprintf('Znalezione przejścia przez setpoint (error: + -> -):\n');
for i = 1:length(zero_crossings)
    fprintf('  Przejście %d: t = %.4f s\n', i, zero_crossings(i));
end

%% Obliczenie okresów
if length(zero_crossings) >= 2
    periods = diff(zero_crossings);
    
    fprintf('\nOkresy między przejściami:\n');
    for i = 1:length(periods)
        fprintf('  T%d = %.4f s\n', i, periods(i));
    end
    
    Tu = mean(periods);
    Tu_std = std(periods);
    
    fprintf('\n=== WYNIK ===\n');
    fprintf('Średni okres oscylacji Tu = %.4f s\n', Tu);
    fprintf('Odchylenie standardowe = %.4f s\n', Tu_std);
    fprintf('Częstotliwość oscylacji f = %.4f Hz\n', 1/Tu);
else
    fprintf('\nZa mało przejść przez zero do obliczenia okresu!\n');
    Tu = NaN;
end

%% Obliczenie parametrów PID metodą Zieglera-Nicholsa
% Tu - okres oscylacji
% Ku - wzmocnienie krytyczne (należy podać!)

fprintf('\n=== METODA ZIEGLERA-NICHOLSA ===\n');
Ku = input('Podaj wzmocnienie krytyczne Ku (Kp przy którym uzyskano oscylacje): ');

if ~isempty(Ku) && Ku > 0
    % Wzory dla regulatora PID (klasyczna metoda Z-N)
    Kp_zn = 0.6 * Ku;
    Ti_zn = 0.5 * Tu;
    Td_zn = 0.125 * Tu;
    
    Ki_zn = Kp_zn / Ti_zn;
    Kd_zn = Kp_zn * Td_zn;
    
    fprintf('\nParametry PID (metoda klasyczna Z-N):\n');
    fprintf('  Kp = %.4f\n', Kp_zn);
    fprintf('  Ki = %.5f\n', Ki_zn);
    fprintf('  Kd = %.2f\n', Kd_zn);
    fprintf('  Ti = %.4f s\n', Ti_zn);
    fprintf('  Td = %.4f s\n', Td_zn);
    
    % Metoda "some overshoot" (mniejsze przeregulowanie)
    fprintf('\nParametry PID (mniejsze przeregulowanie):\n');
    Kp_so = 0.33 * Ku;
    Ti_so = 0.5 * Tu;
    Td_so = 0.33 * Tu;
    Ki_so = Kp_so / Ti_so;
    Kd_so = Kp_so * Td_so;
    
    fprintf('  Kp = %.4f\n', Kp_so);
    fprintf('  Ki = %.5f\n', Ki_so);
    fprintf('  Kd = %.2f\n', Kd_so);
    
    % Metoda "no overshoot"
    fprintf('\nParametry PID (bez przeregulowania):\n');
    Kp_no = 0.2 * Ku;
    Ti_no = 0.5 * Tu;
    Td_no = 0.33 * Tu;
    Ki_no = Kp_no / Ti_no;
    Kd_no = Kp_no * Td_no;
    
    fprintf('  Kp = %.4f\n', Kp_no);
    fprintf('  Ki = %.5f\n', Ki_no);
    fprintf('  Kd = %.2f\n', Kd_no);
end

%% Zaznaczenie przejść na wykresie
figure(1);
subplot(2,1,1);
hold on;
for i = 1:length(zero_crossings)
    xline(zero_crossings(i), 'm--', sprintf('T%d', i), 'LineWidth', 1);
end
