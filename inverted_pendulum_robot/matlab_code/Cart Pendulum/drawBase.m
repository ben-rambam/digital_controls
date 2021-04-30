% Austin Kaul , Jonathan Matthews
% Control Systems Lab 2 (ME 453)
% Dr. Hoover
% Due October 25, 2017

%This function draws a slider base that varies in location based on input
%This function was made to be called by the drawPendulum function.  
% Once the handle input is not empty this function just updates the figure
%handle with the set command.

%Inputs
%   y =      the location of the slider on the track
%   widgth = the width of the slider
%   height = height of the slider
%   handle=  inputs a figure handle

%Outputs
%   handle = the figure handle drawn for the base

function handle = drawBase(y,width,height,handle)

%%%% Define and draw the pendulum cart %%%%%
X = [y-width/2,y-width/2,y+width/2,y+width/2];
Y = [0,height,height,0];

if isempty(handle)
%   handle = plot(X,Y,'g','linewidth',2);
   handle = fill(X,Y,'r');
else
    %Updates Figure with new positions
    set(handle, 'XData',X,'YData',Y);
end
end