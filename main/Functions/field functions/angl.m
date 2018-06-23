function [deg] = angl(P)
dX = diff(P(:,1));
deg = atand(diff(P(:,2))/abs(dX));
if dX < 0
   deg = -180 - deg; 
end
if deg<-180
    deg = deg + 360;
elseif deg > 180
    deg = deg - 360;
end
end

