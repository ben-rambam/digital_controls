clear variables
x = [0;0;0;0];
u = 1;

out = (testFunc(x,u) - linearizedDynamics(x,u))./testFunc(x,u)
trueout = 100*out;

thetas = linspace(-3.14159,3.14159);
count = 1;

for theta = thetas
    x = [0;0;theta;0];
    u = 0;
    nonlinout = testFunc(x,u);
    linout = linearizedDynamics(x,u);
    
    xout1(count) = nonlinout(1);
    xout2(count) = linout(1);
    xdotout1(count) = nonlinout(2);
    xdotout2(count) = linout(2);
    thetaout1(count) = nonlinout(3);
    thetaout2(count) = linout(3);
    thetadotout1(count) = nonlinout(4);
    thetadotout2(count) = linout(4);
    count = count+1;
end
figure(1)
subplot(2,2,1);
plot(thetas, xout1);
hold on;
plot(thetas, xout2);
title('Theta VS Xdot');
legend('Actual', 'Linearized')
xlabel('Theta');
ylabel('Xdot');

subplot(2,2,2);
plot(thetas, xdotout1);
hold on;
plot(thetas, xdotout2);
title('Theta VS Xdoubledot');
legend('Actual', 'Linearized')
xlabel('Theta');
ylabel('XDoubleDot');

subplot(2,2,3);
plot(thetas, thetaout1);
hold on;
plot(thetas, thetaout2);
title('Theta VS Thetadot');
legend('Actual', 'Linearized')
xlabel('Theta');
ylabel('ThetaDot');

subplot(2,2,4);
plot(thetas, thetadotout1);
hold on;
plot(thetas, thetadotout2);
title('Theta VS Thetadoubledot');
legend('Actual', 'Linearized')
xlabel('Theta');
ylabel('Thetadoubledot');


xdots = linspace(-10,10);
count = 1;
for xdot = xdots;
    x = [0;xdot;0;0];
    u = 0;
    nonlinout = testFunc(x,u);
    linout = linearizedDynamics(x,u);

    xout1(count) = nonlinout(1);
    xout2(count) = linout(1);
    xdotout1(count) = nonlinout(2);
    xdotout2(count) = linout(2);
    thetaout1(count) = nonlinout(3);
    thetaout2(count) = linout(3);
    thetadotout1(count) = nonlinout(4);
    thetadotout2(count) = linout(4);

    count = count+1;
end
figure(2)
subplot(2,2,1);
plot(xdots, xout1);
hold on;
plot(xdots, xout2);
title('Xdot VS X');
legend('Actual', 'Linearized')
xlabel('Xdot');
ylabel('Xdot');

subplot(2,2,2);
plot(xdots, xdotout1);
hold on;
plot(xdots, xdotout2);
title('Xdot VS Xdoubledot');
legend('Actual', 'Linearized')
xlabel('Xdot');
ylabel('XDoubleDot');

subplot(2,2,3);
plot(xdots, thetaout1);
hold on;
plot(xdots, thetaout2);
title('Xdot VS Thetadot');
legend('Actual', 'Linearized')
xlabel('Xdot');
ylabel('ThetaDot');

subplot(2,2,4);
plot(xdots, thetadotout1);
hold on;
plot(xdots, thetadotout2);
title('Xdot VS Thetadoubledot');
legend('Actual', 'Linearized')
xlabel('Xdot');
ylabel('Thetadoubledot');

uins = linspace(-10,10);
count = 1;
for uin = uins;
    x = [0;0;0;0];
    u = uin;
    nonlinout = testFunc(x,u);
    linout = linearizedDynamics(x,u);

    xout1(count) = nonlinout(1);
    xout2(count) = linout(1);
    xdotout1(count) = nonlinout(2);
    xdotout2(count) = linout(2);
    thetaout1(count) = nonlinout(3);
    thetaout2(count) = linout(3);
    thetadotout1(count) = nonlinout(4);
    thetadotout2(count) = linout(4);

    count = count+1;
end
figure(3)
subplot(2,2,1);
plot(uins, xout1);
hold on;
plot(uins, xout2);
title('U VS X');
legend('Actual', 'Linearized')
xlabel('U');
ylabel('Xdot');

subplot(2,2,2);
plot(uins, xdotout1);
hold on;
plot(uins, xdotout2);
title('U VS Xdoubledot');
legend('Actual', 'Linearized')
xlabel('U');
ylabel('XDoubleDot');

subplot(2,2,3);
plot(uins, thetaout1);
hold on;
plot(uins, thetaout2);
title('U VS Thetadot');
legend('Actual', 'Linearized')
xlabel('U');
ylabel('ThetaDot');

subplot(2,2,4);
plot(uins, thetadotout1);
hold on;
plot(uins, thetadotout2);
title('U VS Thetadoubledot');
legend('Actual', 'Linearized')
xlabel('U');
ylabel('Thetadoubledot');