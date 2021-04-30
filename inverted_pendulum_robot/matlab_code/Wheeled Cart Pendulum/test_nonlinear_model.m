close all

masstotal = 1.03;
mwheel = 0.038;
b = 0;
L = 49*2.54*0.01;
g = 9.81;
Lcg = 0.28;
R = 0.0325;

m1 = masstotal-2*mwheel;
m2 = 2*mwheel;
I1 = 1/12*m1*L^2;
I2 = 1/2*m2*R^2;
% M = [m(3)+m(1)+m(2), L(1)*m(2)+L(1)*m(1), l(2)*m(2);...
%     1/g, (L(1)^2*m(2)+l(1)^2*m(1)+I(1))/g/(L(1)*m(2)+l(1)*m(1)), L(1)*l(1)*m(2)/g/(L(1)*m(2)+l(1)*m(1));...
%     1/g, L(1)*l(1)/g/l(2), (l(2)^2*m(2) + I(2))/(g*m(2)*l(2))]
% 
% Minv = inv(M);
% 
% A = [ zeros(3,1), Minv(1:3,2:3), zeros(3,3); zeros(3,1), zeros(3,2), eye(3,3) ]
% 
% B = [ Minv(1:3,1); zeros(3,1) ]
% 
% return

% m1 = 1;
% m2 = 1000;
% I1 = 1/12*m1*L^2;
% I2 = 10000;

% fun = @(t,x) nonLinearDynamics_JEM(x, uFun(t), m1, m2, I1, I2, Lcg, R);
fun = @(t,x) testFunc(x, uFun(t));


x0 = [0;0;0.5;0];
[t,x] = ode45(fun, [0 10], x0);

% plot(t,x(1))

tic
for i = 1:length(t)-1
    timestamp = toc;
%     if( timestamp < t(i+1) | t(i) == 0 )
        drawPendulum(-x(i,1), x(i,3), t(i), 2*R, 2*R, Lcg)
        pause(t(i+1)-t(i));
        drawnow;
%     end
end
toc



function u = uFun(t)

u = 0;

end