x = [0;0;0;0]
u = 0;
s = tf('s');
T = 1/100;
z = tf('z', T);
%gs = linearizedDynamics(x,u)
Gs = -0.9306/(s^2 - 8.882e-16*s - 20.37)

MotorGain = -0.00224404;
Gtmp = zpk(Gs*MotorGain)

Gd = c2d(Gtmp, T, 'zoh');

controlSystemDesigner('rlocus', Gd);
Gc = zpk(-160*Gs*(1+0.24*s)/(1+0.01*s))
Gc2 = zpk(7000*Gs*(1+0.2*s)*(1+0.2*s)/((1+0.0033*s)*(1+2*s)))
Gol = -160*Gs*(1+0.24*s)/(1+0.01*s)
Go2 = 7000*Gs*(1+0.2*s)*(1+0.2*s)/((1+0.0033*s)*(1+2*s))
