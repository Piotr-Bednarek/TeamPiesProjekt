clear;
close all;
clc;

s= tf('s');

%Ti=1;
%Kp=1;
%Td=1;



A_stopnie = 9;        

data = readmatrix('wychylenie-109.csv'); 
t_raw = data(:, 1);     % Czas
y_raw = data(:, 3);     % Pozycja piłki [mm]
figure(1)
plot(t_raw, y_raw); title('Zależność pozycji piłki od czasu (A = 40^\circ)');
xlabel('Czas (s)')
ylabel('Pozycja piłki (mm)')


idx = t_raw > 1.35 & t_raw < 3.095; 

t = t_raw(idx);
y = y_raw(idx);

% Przesunięcie czasu i pozycji do 0, żeby parabola wychodziła z punktu
% (0,0)

t = t - t(1);
y = y - y(1);


%Dopsaowanie do wielomianu 2 stopnia
p = polyfit(t, y, 2);

% Współczynnik przy t^2 to nasze p(1).
% Z fizyki wiemy, że y = 0.5 * (K * A) * t^2
% Zatem: p(1) = 0.5 * K * A
K_obl = (2 * p(1)) / A_stopnie; % Jeśli model ma brać stopnie

fprintf('Współczynnik paraboli a (przy t^2): %.4f\n', p(1));
fprintf('Wyliczone wzmocnienie K: %.4f\n', K_obl);

y_model = p(1)*t.^2 + p(2)*t + p(3);

figure(2)
plot(t, y, 'b.', t, y_model, 'r-', 'LineWidth', 2);
xlabel('Czas (s)')
ylabel('Pozycja piłki (mm)')
legend('Dane pomiarowe', 'Dopasowana parabola');
title(['Dopasowanie modelu K/s^2 dla A = 40^\circ.    K = ' num2str(K_obl)]);
grid on;


G= K_obl/(s^(2));
% sys= ss(G)
% 
% A= sys.A
% B= sys.B
% C= sys.C