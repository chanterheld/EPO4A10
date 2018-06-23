function [Field] = fillout(Field,Start)
[H,W] = size(Field);
if (Start(1)<1||Start(1)>H||Start(2)<1||Start(2)>W)
    return 
end
    
    checks = [  1, 0;
            -1, 0;
            0, 1;
            0, -1];
    
    Field(Start(1), Start(2)) = 1;
    cor = [Start(1), Start(2)];
    cornext = zeros(100,2);
    next_ind = 1;

while true
    for i = 1:size(cor,1)
        for k = 1:size(checks,1)
           tmpL =  cor(i,:) + checks(k,:);
           if (tmpL(1)<1||tmpL(1)>H||tmpL(2)<1||tmpL(2)>W)
                continue 
           end
           if Field(tmpL(1), tmpL(2)) == 0
               Field(tmpL(1), tmpL(2)) = 1;
               cornext(next_ind,:) = tmpL;
               next_ind = next_ind + 1;
           end
        end
    end
    if next_ind == 1
       return 
    end 
    cor = cornext(1:next_ind-1,:);
    next_ind = 1;
end
end

