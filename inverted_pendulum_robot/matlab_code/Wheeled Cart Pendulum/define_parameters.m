masstotal = 1.03;
mwheel = 0.038;
b = 0;
m = masstotal-2*mwheel
M = 2*mwheel
L = 49*2.54*0.01
g = 9.81
I=1/12*m*L^2 %Moment of inertia based on the mass and length of rod
r = 0.28
R = 0.0325

m1 = masstotal-2*mwheel;
m2 = 2*mwheel;
I1 = 1/12*m1*L^2;
I2 = 1/2*m2*R^2;