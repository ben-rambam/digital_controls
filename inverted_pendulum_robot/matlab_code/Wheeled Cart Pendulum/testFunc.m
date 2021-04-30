function xdot=testFunc(x,u)
% x(1) = x
% x(2) = xdot
% x(3) = theta
% x(4) = theta dot
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
% b = 0;
% m = 1;
% M = 1;
% L = 1;
% g = -9.81;
% I=1/12*m*L^2; %Moment of inertia based on the mass and length of rod
% r = L/2;
% R = 0.0325; %radius of wheel
%Equations used for the dynamics of the system, found by solving for
%thetadoubledot and then substituting to find xdoubledot
thetatop = -m*g*r*sin(x(3))-(m*r*cos(x(3)))/(M*m)*(u*R-b*x(2)+m*r*x(4)^2*sin(x(3)));
thetabottom = (I+m*r^2)+(m*r*cos(x(3))*(-m*r*cos(x(3))))/(M+m);
thetacombined = thetatop/thetabottom;
%System of first order differential equations that describes the dynamics
%of the pendulum cart system
xdot(1) = x(2);
xdot(2) = (u*R-b*x(2)-m*r*thetacombined*cos(x(3))+m*r*x(4)^2*sin(x(3)))/(M+m);
xdot(3) = x(4);
xdot(4) = thetacombined;
xdot = xdot';
end

