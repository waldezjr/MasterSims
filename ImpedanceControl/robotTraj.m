%radius 0.1 centered at [0;0.4] phase pi/2
function [xd]=robotTraj(t,omega,tSim)
    if t<tSim
        xd = [0.1*cos(omega*t+pi/2);0.4+0.1*sin(omega*t+pi/2)];
    else
        xd = [0;0.5];
    end

end