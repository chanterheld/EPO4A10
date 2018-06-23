function [field] = place_points(field, points)
[H,W] = size(field);
     for i = 1:size(points,1)
         if (points(i,1)<1||points(i,1)>H||points(i,2)<1||points(i,2)>W)
            continue
         end
        field(points(i,1),points(i,2)) = 1;
     end
end

