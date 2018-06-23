function [waypoints,failed] = rout(Start,Finish, Field,Neighboors,Hn,radius,maxdev,first_point)

add_fp = 0;
if Field(Start(1),Start(2)) == 1
    Start = first_point;
    add_fp = 1;
end

failed = 0;
path = [];
[Height,Width]=size(Field);
G_score=inf(Height,Width);
F_score=single(inf(Height,Width));
Open_set=int8(zeros(Height,Width)); %discoverd & !evaluated
Closed_set =int8(zeros(Height,Width)); %evaluated or object
Closed_set(Field==1)=1;
orgX =int16(zeros(Height,Width));
orgY =int16(zeros(Height,Width));

F_score(Start(1),Start(2)) = Hn(Start(1),Start(2));
G_score(Start(1),Start(2)) = 0;
Open_set(Start(1),Start(2)) = 1;

while true
    minF = min(min(F_score));
    if minF == inf
        failed = 1;
        break; %no path
    end
    [X, Y] = find(F_score == minF);
    X = X(1);
    Y = Y(1);
    
    if((X == Finish(1))&&(Y == Finish(2)))
       break; %done
    end
    
    Open_set(X,Y) = 0;
    Closed_set(X,Y) = 1;
    F_score(X,Y) = inf;
    
    for n = 1:size(Neighboors,1)
        dX = Neighboors(n,1);
        dY = Neighboors(n,2);
        if (X+dX < 1)||(X+dX > Height)||(Y+dY < 1)||(Y+dY > Width)
            continue %outside field
        end
        
        if Closed_set(X+dX, Y+dY) == 0
            crossed = 0;
            if (abs(dX)>1||abs(dY)>1)

                cellCr=max(abs(dX),abs(dY));
                for K=1:cellCr
                    Yfr=round(K*dY/cellCr);
                    Xfr=round(K*dX/cellCr);
            
                    if(Field(X+Xfr,Y+Yfr)==1)
                        crossed = 1;
                        continue
                    end
                end
            end
            if crossed
               continue
               disp('crossed')
            end
            newG = G_score(X,Y) + sqrt(dX^2 + dY^2);
            Open_set(X+dX,Y+dY) = 1;
            if newG >= G_score(X+dX,Y+dY)
                continue
            end
            orgX(X+dX,Y+dY) = X;
            orgY(X+dX,Y+dY) = Y;
            G_score(X+dX,Y+dY) = newG;
            F_score(X+dX,Y+dY) = newG + Hn(X+dX,Y+dY);
            
        end        
    end
end

if ~failed
    path(1,:) = [X Y];
    ind = 2;
    while true
        path(ind,1) = orgX(X,Y);
        path(ind,2) = orgY(X,Y);
        X = path(ind,1);
        Y = path(ind,2);
        if X == Start(1)&&Y == Start(2)
            break 
        end
        ind = ind + 1;
    end
    path = flipud(path);
    
    anglemat = zeros(1,ind-1);
    for i = 1:ind-1
        anglemat(i) = angl(path(i:i+1,:));
    end

    angl_intr = 0;
    waypoints = zeros(ind,2);
    waypoints(1,:) = path(1,:);
    ind2 = 1;
    
    for i = 1:ind-2
        err = diff(anglemat(i:(i+1)));
        angl_intr = 1.3*(angl_intr) + err;
        if (abs(err) > 20)||(abs(angl_intr)>30)
            ind2 = ind2+1;
            angl_intr = 0;
            waypoints(ind2,:) = path(i+1,:);       
        end
    end
    

    waypoints = [waypoints(1:ind2,:);path(end,:)];
    
    for i = 2:size(waypoints,1)-1
        inn_angl = 180 - abs(angl([waypoints(i,1:2);waypoints(i+1,1:2)]) - angl([waypoints(i-1,1:2);waypoints(i,1:2)]));
        waypoints(i,3) = min(max(.15,(radius/tand(inn_angl/2))), sqrt(2*radius*maxdev+maxdev^2));
    end
    waypoints(end,3) = .2;
    
    if ~add_fp
    waypoints = waypoints(2:end,:);
    else
        waypoints(1,3) = .2;
    end
    
else
    disp('routing failed')
    waypoints = Finish;
end


end





