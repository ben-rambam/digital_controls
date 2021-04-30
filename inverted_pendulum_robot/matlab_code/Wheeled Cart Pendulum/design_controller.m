Gdyn = linearizedDynamics([0;0;0;0],[0])
Gmot = -0.0022

Gs = Gmot*Gdyn

f = 100
T = 1/f

Gd = c2d(Gs, T, 'zoh')

controlSystemDesigner(Gd)