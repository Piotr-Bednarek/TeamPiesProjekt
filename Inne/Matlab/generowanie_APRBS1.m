clear; clc; close all;

%% 1. Konfiguracja (Dostosowana do krótkiej belki 25cm)
% Automatyczne pobranie czasu próbkowania z main.h
main_h_path = '..\STM\Core\Inc\main.h';
if exist(main_h_path, 'file')
    txt = fileread(main_h_path);
    tokens = regexp(txt, '#define\s+PID_DT_MS\s+(\d+)', 'tokens');
    if ~isempty(tokens)
        dt_ms = str2double(tokens{1}{1});
        fprintf('Wykryto PID_DT_MS = %d ms z main.h\n', dt_ms);
        Ts = dt_ms / 1000;
        Fs = 1/Ts;
    else
        warning('Nie znaleziono definicji PID_DT_MS. Przyjęto domyślnie 30ms.');
        Ts = 0.03;
        Fs = 1/Ts;
    end
else
    warning('Nie znaleziono pliku main.h! Przyjęto domyślnie 30ms.');
    Ts = 0.03;
    Fs = 1/Ts;
end

T_total = 10;           % Czas trwania eksperymentu
N = round(T_total * Fs); % Liczba próbek

% --- Parametry Serwa (Skala 0-200 stopni -> 500-2500us) ---
pwm_center = 1500;      % [us] Poziom (100 stopni w skali 0-200)
pwm_range = 90;         % [us] +/- 12 stopni (bezpieczne dla 25cm)

% Czasy trwania impulsów (Bardzo krótkie, żeby nie uderzyć w ścianę)
min_hold_time = 0.4;   % [s] Szybkie kontry
max_hold_time = 0.65;   % [s] Max czas lotu w jedną stronę

%% 2. Generowanie sygnału Naprzemiennego (Ping-Pong)
t = (0:N-1)' * Ts;
u_pwm = zeros(N, 1);

current_idx = 1;
direction = -1; % Zmienna decydująca o kierunku (1 lub -1)

% Zacznij od zera przez chwilę (bezpieczny start)
start_delay = 1.0; % [s]
start_samples = round(start_delay/Ts);
u_pwm(1:start_samples) = pwm_center;
current_idx = start_samples + 1;

while current_idx <= N
    % 1. Losuj czas trwania tego ruchu
    hold_samples = randi([round(min_hold_time/Ts), round(max_hold_time/Ts)]);
    
    % 2. Losuj amplitudę (zawsze > 0, bo znak dodajemy z 'direction')
    % Zakres od 50% do 100% pwm_range, żeby nie było zbyt słabych ruchów
    amplitude = (0.5 + 0.5 * rand) * pwm_range; 
    
    % 3. Oblicz wartość PWM z uwzględnieniem ZMIANY KIERUNKU
    val = round(pwm_center + (direction * amplitude));
    
    % 4. Zapisz do tablicy
    end_idx = min(current_idx + hold_samples - 1, N);
    u_pwm(current_idx:end_idx) = val;
    
    % 5. Przygotuj kolejny krok
    current_idx = end_idx + 1;
    direction = -direction; % <--- KLUCZ: Zawsze zmieniaj znak na przeciwny!
end

%% 4. Eksport do plików .c i .h w projekcie STM32
% Ścieżki względne do folderów STM32
output_dir_src = '..\STM\Core\Src\';
output_dir_inc = '..\STM\Core\Inc\';

% Sprawdzenie czy foldery istnieją (dla bezpieczeństwa)
if ~exist(output_dir_src, 'dir') || ~exist(output_dir_inc, 'dir')
    error('Nie znaleziono folderów projektu STM32! Uruchom skrypt z folderu Matlab.');
end

file_c = fullfile(output_dir_src, 'signal.c');
file_h = fullfile(output_dir_inc, 'signal.h');

% --- Generowanie signal.h ---
fid = fopen(file_h, 'w');
fprintf(fid, '#ifndef INC_SIGNAL_H_\n');
fprintf(fid, '#define INC_SIGNAL_H_\n\n');
fprintf(fid, '#include <stdint.h>\n\n');
fprintf(fid, 'extern const uint16_t pwm_sequence[%d];\n', N);
fprintf(fid, 'extern const uint32_t SEQUENCE_LEN;\n\n');
fprintf(fid, '#endif /* INC_SIGNAL_H_ */\n');
fclose(fid);
fprintf('Wygenerowano: %s\n', file_h);

% --- Generowanie signal.c ---
fid = fopen(file_c, 'w');
fprintf(fid, '#include "signal.h"\n\n');
fprintf(fid, 'const uint32_t SEQUENCE_LEN = %d;\n\n', N);
fprintf(fid, 'const uint16_t pwm_sequence[%d] = {\n', N);

for i = 1:N
    if i == N
        fprintf(fid, '  %d', u_pwm(i));
    elseif mod(i, 10) == 0
        fprintf(fid, '  %d,\n', u_pwm(i));
    else
        fprintf(fid, '  %d, ', u_pwm(i));
    end
end
fprintf(fid, '\n};\n');
fclose(fid);
fprintf('Wygenerowano: %s\n', file_c);

% Zapisz też same dane do pliku .mat dla późniejszej identyfikacji
save('dane_wymuszenia.mat', 'u_pwm', 't', 'Ts');