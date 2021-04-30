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

x0 = [0; 0 ; 1.5; 0]; % define initial conditions
str = []; % str is always an empty matrix
% initialize the array of sample times
ts = [0 0]; % continuous sample time
end

%============================================================================
function xdot=mdlDerivatives(t,x,u,m,g,L,M,b)
% masstotal = 1.03;
% mwheel = 0.038;
% b = 0;
% L = 49*2.54*0.01;
% g = 9.81;
% Lcg = 0.28;
% R = 0.0325;
% 
% m1 = masstotal-2*mwheel;
% m2 = 2*mwheel;
% I1 = 1/12*m1*L^2;
% I2 = 1/2*m2*R^2;
% 
% xdot = nonLinearDynamics_JEM(x, u, m1, m2, I1, I2, Lcg, R);

xdot = testFunc(x,u);

end

%============================================================================
function sys=mdlOutputs(t,x,w,h,L)
sys(1) = x(1);%output x of cart
sys(2) = x(3);%output theta of pendulum
sys(3) = w; % width of box
sys(4) = h; % height of box
sys(5) = L; % Length of rod

end




