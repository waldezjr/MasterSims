function [xEq,xEqDot,xEqDotDot]=minJerkProfile(t,T,x0,xD)
        
        xEq =       x0 +(xD-x0)*(10*(t^3)/T^3 -15*(t^4)/T^4 +6*(t^5)/T^5);
        xEqDot =        (xD-x0)*(30*(t^2)/T^3 -60*(t^3)/T^4 +30*(t^4)/T^5);
        xEqDotDot =     (xD-x0)*(60*(t^1)/T^3 -180*(t^2)/T^4 +120*(t^3)/T^5);
        
end