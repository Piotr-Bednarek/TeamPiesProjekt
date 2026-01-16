clear; clc; close all;

%% 1. Konfiguracja parametrów
Fs = 50;                % Częstotliwość serwa (50 Hz)
Ts = 1/Fs;              % Czas próbkowania (0.02 s)
T_total = 10;           % Czas trwania jednej sekwencji (sekundy)
N = T_total * Fs;       % Całkowita liczba próbek

% --- Parametry Serwa (dla zakresu 0.5ms - 2.5ms) ---
pwm_center = 1500;      % [us] Środek zakresu (teoretyczne 0 stopni / poziom)
                        % WAŻNE: Tu wpisz wartość, przy której belka jest w poziomie!

% Ograniczenie wychyleń dla identyfikacji (Mały zakres!)
% Nie chcemy zakresu 50-150 stopni, bo kulka spadnie w 0.2 sekundy.
% Celujemy w np. +/- 10 do 20 stopni wokół poziomu.
pwm_range = 70;        % [us] Amplituda wahań (np. 150us to ok. 15-20 stopni)
                        % Zakres sygnału będzie: [1350us ... 1650us]

min_hold_time = 0.2;    % [s] Minimalny czas trwania jednego "schodka"
max_hold_time = 0.6;    % [s] Maksymalny czas trwania jednego "schodka"

%% 2. Generowanie sygnału losowego (Schodkowego)
t = (0:N-1)' * Ts;      % Wektor czasu
u_pwm = zeros(N, 1);    % Wektor sygnału sterującego (w mikrosekundach)

current_idx = 1;
while current_idx <= N
    % Losuj czas trwania tego schodka (w próbkach)
    hold_samples = randi([round(min_hold_time/Ts), round(max_hold_time/Ts)]);
    
    % Losuj amplitudę dla tego schodka (w zakresie +/- pwm_range)
    % rand daje (0-1), więc (rand-0.5)*2 daje (-1 do 1)
    amplitude = (rand - 0.5) * 2 * pwm_range; 
    
    val = round(pwm_center + amplitude);
    
    % Zapisz do wektora (zabezpieczenie przed wyjściem poza tablicę)
    end_idx = min(current_idx + hold_samples - 1, N);
    u_pwm(current_idx:end_idx) = val;
    
    current_idx = end_idx + 1;
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

%% 4. Eksport do formatu C (do wklejenia do STM32)
% Tworzymy gotowy string z tablicą uint16_t
fprintf('\n--- SKOPIUJ PONIŻSZY KOD DO STM32 ---\n\n');
fprintf('const uint16_t pwm_sequence[%d] = {\n', N);

for i = 1:N
    if i == N
        fprintf('%d', u_pwm(i)); % Ostatni element bez przecinka
    elseif mod(i, 10) == 0
        fprintf('%d,\n', u_pwm(i)); % Nowa linia co 10 liczb dla czytelności
    else
        fprintf('%d, ', u_pwm(i));
    end
end
fprintf('\n};\n');
fprintf('\n--- KONIEC KODU ---\n');

% Zapisz też same dane do pliku .mat dla późniejszej identyfikacji
save('dane_wymuszenia.mat', 'u_pwm', 't', 'Ts');