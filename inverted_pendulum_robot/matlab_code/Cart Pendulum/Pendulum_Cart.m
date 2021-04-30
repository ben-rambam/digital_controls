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

x0 = [0; 0 ; pi*.6 ; 0]; % define initial conditions
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

I=1/12*m*L^2; %Moment of inertia based on the mass and length of rod
r = L/2;
%Equations used for the dynamics of the system, found by solving for
%thetadoubledot and then substituting to find xdoubledot
thetatop = -m*g*r*sin(x(3))-(m*r*cos(x(3)))/(M*m)*(u-b*x(2)+m*r*x(4)^2*sin(x(3)));
thetabottom = (I+m*r^2)+(m*r*cos(x(3))*(-m*r*cos(x(3))))/(M+m);

thetacombined = thetatop/thetabottom;


%System of first order differential equations that describes the dynamics
%of the pendulum cart system
xdot(1) = x(2);
xdot(2) = (u-b*x(2)-m*r*thetacombined*cos(x(3))+m*r*x(4)^2*sin(x(3)))/(M+m);
xdot(3) = x(4);
xdot(4) = thetacombined;
end

%============================================================================
function sys=mdlOutputs(t,x,w,h,L)
sys(1) = x(1);%output x of cart
sys(2) = x(3);%output theta of pendulum
sys(3) = w; % width of box
sys(4) = h; % height of box
sys(5) = L; % Length of rod

end




