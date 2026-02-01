clear all;
%APRBS

Ts=0.6766;
K2 =11.67;
K1= K2;
To= 0.3;
tau= 0.15;


%Mz(s)= (s+beta*ksi*wn)(s^(2)+2*ksi*wn*s+wn^(2))
%czyli człon oscylacyjny i biegun znacznie mniej istotny z współczynnikiem
%beta odsuwającym go od 0.

ksi= 0.707;
beta = 3;
wn= 1.96;
%tutaj jest to wynik rozwiązania jakiegoś wielomianu, bo generalnie
%w tej metodzie mamy 2 parametry do policzenia, a 3 wartości
%od których są zależne 

Kp = ( (Ts * tau * beta * ksi) / K2 ) * wn^3
Kd = ( (Ts + tau)/(K2 * tau) ) - ( (Ts * ksi * (beta + 2))/K2 ) * wn;

Td= Kd
