% Austin Kaul, Jonathan Mathews
% Lab 2, Control Systems (Dr. Hoover), ME 453
% Dr. Hoover
% Due October 25th, 2017

% This code implements the dynamics neccessary for the inverted pendulum on
% a cart. This code will be called in Simulink using the S-function
% The S-function is very specific on inputs to the system so t, x, u are
% required inputs

%Inputs
%   t,x,u,flax = Inputs required for the S-Function in SIMULINK
%   m = mass of the pendulum
%   g = gravity's acceleration
%   L = Length of the pendulum rod
%   M = Mass of the cart
%   b = coeficient of friction
%   w = width of box
%   h = height of box

%Output
%   Sys = the outputs of the system which are position of cart and angle of
%         pendulum
%   x0 = initial states of the system
%   str,ts = outputs required by the S-Function in Simulink

function [sys,x0,str,ts] = Pendulum_Cart(t,x,u,flag,m,g,L,M,b,w,h)

switch flag,
case 0,
[sys,x0,str,ts]=mdlInitializeSizes; % initialize block
case 1,
sys=mdlDerivatives(t,x,u,m,g,L,M,b); % define dynamics
case 3,
sys=mdlOutputs(t,x,w,h,L);
otherwise,
sys = [];
end
end

%============================================================================
function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates = 4;
sizes.NumDiscStates = 0;
sizes.NumOutputs = 5;
sizes.NumInputs = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1; % at least one sample time is needed
sys = simsizes(sizes);

x0 = [0; 0 ; 1.57; 0]; % define initial conditions
str = []; % str is always an empty matrix
% initialize the array of sample times
ts = [0 0]; % continuous sample time
end

%============================================================================
function xdot=mdlDerivatives(t,x,u,m,g,L,M,b)
% x(1) = x
% x(2) = xdot
% x(3) = theta
% x(4) = theta dot

%m = [100,1] %initialize masses
%I(1)=1/12*m(1)*L^2; %Moment of inertia based on the mass and length of rod
%I(2)=0; %consider inertia of wheel to be neglegible
%R = L/2;
%Equations used for the dynamics of the system, found by solving forc
%thetadoubledot and then substituting to find xdoubledot
%A = m(1)*L
%B = m(1)+m(2)+I(2)/R^2
%C = A*L
%tau = 0; %u(1)
%thetadoubledot = (-x(4)^2*cos(x(3))*sin(x(3))*(A^2/B-C)+A*-g*sin(x(3))+tau)/...
%                    (-A^2/B*cos(x(3)^2) + C*cos(x(3))^2 + I(1));
%xdoubledot = (A*(x(4)^2)*sin(x(3)) - A*thetadoubledot*cos(x(3)))/B

%adjust state of x to next state

%xdot(1) = x(2)
%xdot(2) = xdoubledot
%xdot(3) = x(4)
%xdot(4) = thetadoubledot

masstotal = 1.03;
mwheel = 0.038;
b = 0;
m = masstotal-2*mwheel;
M = 2*mwheel;
L = 49*2.54*0.01;
g = 9.81;
I=1/12*m*L^2 %Moment of inertia based on the mass and length of rod
r = 0.28;
R = 0.0325;

m1 = masstotal-2*mwheel;
m2 = 2*mwheel;
I1 = 1/12*m1*L^2;
I2 = 1/2*m2*R^2;
Lcg = 0.28;

m1 = 100;
m2 = 0.1;
I1 = 0.1;
I2 = 0.1;


%Equations used for the dynamics of the system, found by solving for
%thetadoubledot and then substituting to find xdoubledot
% thetatop = -m*g*r*sin(x(3))-(m*r*cos(x(3)))/(M*m)*(u*R-b*x(2)+m*r*x(4)^2*sin(x(3)));
% thetabottom = (I+m*r^2)+(m*r*cos(x(3))*(-m*r*cos(x(3))))/(M+m);
% 
% thetacombined = thetatop/thetabottom;



%System of first order differential equations that describes the dynamics
%of the pendulum cart system
% xdot(1) = x(2);
% xdot(2) = (u*R-b*x(2)-m*r*thetacombined*cos(x(3))+m*r*x(4)^2*sin(x(3)))/(M+m);
% xdot(3) = x(4);
% xdot(4) = thetacombined;

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

end

%============================================================================
function sys=mdlOutputs(t,x,w,h,L)
sys(1) = x(1);%output x of cart
sys(2) = x(3);%output theta of pendulum
sys(3) = w; % width of box
sys(4) = h; % height of box
sys(5) = L; % Length of rod

end




