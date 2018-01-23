%radius 0.1 centered at [0;0.4] phase pi/2
function [xd,xddot,xddotdot]=robotTraj(t,omega,tSim)
    if t<tSim
        xd = [0.1*cos(omega*t+pi/2);0.4+0.1*sin(omega*t+pi/2)];
        xddot = [-0.1*omega*sin(omega*t+pi/2);0.1*omega*cos(omega*t+pi/2)];
        xddotdot = [-0.1*omega*omega*cos(omega*t+pi/2);-0.1*omega*omega*sin(omega*t+pi/2)];
    else
        xd = [0;0.5];
        xddot = [0;0];
        xddotdot = [0;0];
    end

end