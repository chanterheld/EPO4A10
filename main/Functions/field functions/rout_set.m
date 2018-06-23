function [Hn,Neighboors] = rout_set(H, W, Finish,Connecting_Distance)
%heuristic matrix generation

Hn=single(zeros(H,W)); 
for k=1:H
    for j=1:W
        path = Finish - [k j];
        Hn(k,j) = sqrt(sum(path.^2,2));
    end
end

%Neighboors calc
NeighboorCheck=ones(2*Connecting_Distance+1);
Dummy=2*Connecting_Distance+2;
Mid=Connecting_Distance+1;
for i=1:Connecting_Distance-1
    NeighboorCheck(i,i)=0;
    NeighboorCheck(Dummy-i,i)=0;
    NeighboorCheck(i,Dummy-i)=0;
    NeighboorCheck(Dummy-i,Dummy-i)=0;
    NeighboorCheck(Mid,i)=0;
    NeighboorCheck(Mid,Dummy-i)=0;
    NeighboorCheck(i,Mid)=0;
    NeighboorCheck(Dummy-i,Mid)=0;
end
NeighboorCheck(Mid,Mid)=0;

[row, col]=find(NeighboorCheck==1);
Neighboors=[row col]-(Connecting_Distance+1);

end

