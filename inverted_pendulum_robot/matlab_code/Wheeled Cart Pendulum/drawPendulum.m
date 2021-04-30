% Austin Kaul , Jonathan Matthews
% Control Systems Lab 2 (ME 453)
% Dr. Hoover
% Due October 25, 2017

%This function calls two other functions that create a rod and a base. This
%rod and base move based on the inputs. This function is normally called in
%simulink with a user defined function operator.

%Inputs
%   y =     the location of the slider on the track
%   theta = the angle at which the rod is rotated
%   t =     the current time
%   width = width of box
%   height = height of box
%   L = length of rod

%Outputs
%   drawPendulum = draws the Slider and rod to make a pendulum that will
%                  move coninuosly in time when called by SIMULINK

function drawPendulum(y, theta, t, width, height, L)

%define persistent variables
persistent base_handle
persistent rod_handle

%first time function is called, initialize plot and persistent vars
if t==0,
    figure(1), clf
    track_width=L+1;
    plot([-track_width,track_width],[0,0],'k'); %plot track
    hold on
    base_handle = drawBase(y,width,height,[]);
    rod_handle = drawRod(y,theta, L,height,[]);
    axis([-track_width,track_width,-track_width,track_width]);
    %at every other time step, redraw base and rod
else
    drawBase(y,width,height,base_handle);
    drawRod(y,theta,L,height,rod_handle);
end
end