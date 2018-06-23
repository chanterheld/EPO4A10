function [points] = draw_disk(mid,radius)
     tempfield = zeros(2*radius + 5);
     tempmid = [radius+3, radius+3];
     middif = mid - tempmid;
     temppoints = drawcircle(tempmid, radius);
     [tempfield] = place_points(tempfield, temppoints);
     [tempfield] = fillout(tempfield,tempmid);
     [x, y] = find(tempfield == 1);
     points = [x + middif(1), y + middif(2)];
end

