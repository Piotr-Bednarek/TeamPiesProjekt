clear;
close all;
clc;

s= tf('s');
%Parametry regulatora
Ti=1;
Kp=1;
Td=1;


% --- KONFIGURACJA ---
A_stopnie = 25;          % Jaki skok zadałeś w eksperymencie (w stopniach)?
%A_rad = deg2rad(A_stopnie); % Przeliczamy na radiany (fizyka lubi radiany!)

% --- WCZYTANIE DANYCH ---
data = readmatrix('wychylenie-125.csv'); 
t_raw = data(:, 1);     % Czas
y_raw = data(:, 3);     % Pozycja kulki [m] (upewnij się że metry, nie mm!)

plot(t_raw, y_raw); title('Znajdź zakres czasu startu i końca');


idx = t_raw > 1.1 & t_raw < 2.2; 

t = t_raw(idx);
y = y_raw(idx);

% Przesuwamy czas i pozycję do zera (start paraboli w (0,0))
t = t - t(1);
y = y - y(1);

% --- OBLICZANIE K (POLYFIT) ---
% Dopasowujemy wielomian 2. stopnia: y = p(1)*t^2 + p(2)*t + p(3)
p = polyfit(t, y, 2);

% Współczynnik przy t^2 to nasze p(1).
% Z fizyki wiemy, że y = 0.5 * (K * A) * t^2
% Zatem: p(1) = 0.5 * K * A

%K_obl = (2 * p(1)) / A_rad;  % Jeśli model ma brać radiany
K_obl = (2 * p(1)) / A_stopnie; % Jeśli model ma brać stopnie

fprintf('Współczynnik paraboli a (przy t^2): %.4f\n', p(1));
fprintf('Wyliczone wzmocnienie K: %.4f\n', K_obl);

% --- WERYFIKACJA ---
y_model = p(1)*t.^2 + p(2)*t + p(3);
figure;
plot(t, y, 'b.', t, y_model, 'r-', 'LineWidth', 2);
legend('Dane pomiarowe', 'Dopasowana parabola');
title(['Dopasowanie modelu K/s^2. K = ' num2str(K_obl)]);
grid on;


G= K_obl/(s^(2));
% sys= ss(G)
% 
% A= sys.A
% B= sys.B
% C= sys.C