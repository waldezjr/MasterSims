function [xd]=humanTraj(t,omega,tSim)
        
    deltaT = tSim/8;
    p1 = robotTraj(deltaT,omega);
    p2 = [-0.1;0.4];
    p3 = robotTraj(3*deltaT,omega);
    p4 = robotTraj(5*deltaT,omega);
    
    if t < deltaT
        xd = robotTraj(t,omega);
        
    elseif t >= deltaT && t < 2*deltaT
        
        xd = (t-deltaT) * (p2-p1)/1.25 + p1;
        
    elseif t >= 2*deltaT && t < 3*deltaT
        
        xd = (t-2*deltaT) * (p3-p2)/1.25 + p2;
        
    elseif t >= 3*deltaT && t < 5*deltaT
        
        xd = (t-3*deltaT) * (p4-p3)/2.5 + p3;
        
    elseif t >= 5*deltaT
        xd = robotTraj(t,omega);        
    end
end