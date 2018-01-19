function [xd]=humanTraj(t,omega,tSim)
        
    deltaT = tSim/8;
    p1 = robotTraj(deltaT,omega,tSim);
    p2 = [-0.15;0.4];
    p3 = robotTraj(3*deltaT,omega,tSim);
    p4 = robotTraj(5*deltaT,omega,tSim);
    
    if t < deltaT
        xd = robotTraj(t,omega,tSim);
        
    elseif t >= deltaT && t < 2*deltaT
        
        xd = (t-deltaT) * (p2-p1)/1.25 + p1;
        
    elseif t >= 2*deltaT && t < 3*deltaT
        
        xd = (t-2*deltaT) * (p3-p2)/1.25 + p2;
        
    elseif t >= 3*deltaT && t < 5*deltaT
        
        xd = (t-3*deltaT) * (p4-p3)/2.5 + p3;
        
    elseif t >= 5*deltaT && t < tSim
        xd = robotTraj(t,omega,tSim);
    elseif t >= tSim
        xd =[0;0.5];
    end
end