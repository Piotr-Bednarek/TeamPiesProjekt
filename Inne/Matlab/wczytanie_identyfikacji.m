data = readmatrix('prbs.csv');
%data1 = readmatrix('identyfikacja-6.csv');

u = data(:,6);  % <-- Tu wpisz numer kolumny z PWM
y = data(:, 3);  % <-- Tu wpisz numer kolumny z ToF

%u1 = data1(:, 4);  % <-- Tu wpisz numer kolumny z PWM
%y1 = data1(:, 3);  % <-- Tu wpisz numer kolumny z ToF

Ts = 0.034;