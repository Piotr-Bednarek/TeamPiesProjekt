%%
% Skok


K_obl=[11.3337, 11.7193, 11.6871, 11.9525];

K1= mean(K_obl);

%Mz(s) = s^(2)+ 2*K*Kd+ K*Kp
%M(s) = s^(2)+ 2*wn*ksi+ wn^(2)

wn1= 4.5;
ksi= 0.707;

Kp= wn1^(2)/K1

Td= (2*ksi*wn1)/K1


Ti=3;





