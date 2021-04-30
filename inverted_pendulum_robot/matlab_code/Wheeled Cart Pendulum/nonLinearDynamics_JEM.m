function xdot = nonLinearDynamics_JEM(x, u, m1, m2, I1, I2, Lcg, R)
g = 9.81;

X = x(1);
Xdot = x(2);
T = x(3);
Tdot = x(4);
tau = u(1);

A = m1*Lcg;
B = m1+m2+I2/R^2;
C = A*Lcg;

thetatop = -Tdot^2 * cos(T) * sin(T) * (A^2/B-C) + A*g*sin(T) + tau;
thetabot = -A^2/B*cos(T)^2 + C*cos(T)^2 + I1;

Tddot = thetatop/thetabot;

Xddottop = m1*Lcg*Tdot^2*sin(T) - m1*Lcg*Tddot*cos(T);
Xddotbot = m1+m2*I2/R^2;
Xddot = Xddottop/Xddotbot;

xdot(1) = Xdot;
xdot(2) = Xddot;
xdot(3) = Tdot;
xdot(4) = Tddot;

xdot = xdot';
end

