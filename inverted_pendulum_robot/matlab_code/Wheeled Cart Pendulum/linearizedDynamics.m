function output=linearizedDynamics(x,u)
masstotal = 1.03;
mwheel = 0.038;
b = 0;
m = masstotal-2*mwheel;
M = 2*mwheel;
L = 49*2.54*0.01;
g = -9.81;
I=1/12*m*L^2; %Moment of inertia based on the mass and length of rod
r = 0.28;
R = 0.0325;
P1 = m*r^2;
P2 = 1+-m/(M+m);
P3 = I + P1*P2;

k21 = 0;
%k22 = (b*m*r/((M+r)*M*m*P3))-(b/(M+m));
%k22 = (-b*m*r/((M+r)*M*m*P3))-(b/(M+m));
k22 = (-b*m*r/((M+m)*M*P3))-(b/(M+m));
%k23 = -m^2*r^2*g/((M+r)*P3);
k23 = (m*r)^2*g/((M+m)*P3);
k24 = 0;
k41 = 0;
k42 = -b/(M*m*P3);
%k43 = m*g*r/P3;
k43 = -m*g*r/P3;
k44 = 0;
b1  = 0;
%b2  = (m*r^2/((M+r)*M*P3))-(R/(M+m));
b2  = R*((m*r^2/((M+m)*M*P3))+(1/(M+m)));
b3  = 0;
b4  = -r*R/(M*P3);
%b4  = r*R/(M*P3);

A = [0,1,0,0;...
    0,k22,k23,0;...
    0,0,0,1;...
    0,k42,k43,0];

B = [0;b2;0;b4];

C = [0,0,1,0];

D = [0];

y = C*x + D*u;
xdot = A*x + B*u;
xtripdot = (xdot);
output = tf(ss(A,B,C,D));

end