function points = drawcircle(mid, radius)
%https://nl.mathworks.com/matlabcentral/fileexchange/33844-circle-pixel-coordinates-using-mid-point-algorithm
%https://en.wikipedia.org/wiki/Midpoint_circle_algorithm
     

     x0 = mid(1);
     y0 = mid(2);

     octant_size = floor((sqrt(2)*(radius-1)+4)/2);
     n_points = 8 * octant_size;

     xc = NaN(n_points, 1);
     yc = NaN(n_points, 1);
     
     x = 0;
     y = radius;
     f = 1 - radius;
     dx = 1;
     dy = - 2 * radius;
     
     % Store
     
     % 1 octant
     xc(1) = x0 + x;
     yc(1) = y0 + y;
     
    % 2nd octant 
     xc(8 * octant_size) = x0 - x;
     yc(8 * octant_size) = y0 + y;
     
     % 3rd octant 
     xc(4 * octant_size) = x0 + x;
     yc(4 * octant_size) = y0 - y;
     
     % 4th octant 
     xc(4 * octant_size + 1) = x0 - x;
     yc(4 * octant_size + 1) = y0 - y;
     
     % 5th octant 
     xc(2 * octant_size) = x0 + y;
     yc(2 * octant_size) = y0 + x;
     
     % 6th octant 
     xc(6 * octant_size + 1) = x0 - y;
     yc(6 * octant_size + 1) = y0 + x;
     
     % 7th octant 
     xc(2 * octant_size + 1) = x0 + y;
     yc(2 * octant_size + 1) = y0 - x;
     
     % 8th octant 
     xc(6 * octant_size) = x0 - y;
     yc(6 * octant_size) = y0 - x;
     
     
     for i = 2 : n_points/8
         
         if f > 0
             y = y - 1;
             dy = dy + 2;
             f = f + dy;
         end
         x = x + 1;
         dx = dx + 2;
         f = f + dx;
         
         % 1 octant
         xc(i) = x0 + x;
         yc(i) = y0 + y;
         
         % 2nd octant
         xc(8 * octant_size - i + 1) = x0 - x;
         yc(8 * octant_size - i + 1) = y0 + y;
         
         % 3rd octant
         xc(4 * octant_size - i + 1) = x0 + x;
         yc(4 * octant_size - i + 1) = y0 - y;
         
         % 4th octant
         xc(4 * octant_size + i) = x0 - x;
         yc(4 * octant_size + i) = y0 - y;
         
         % 5th octant
         xc(2 * octant_size - i + 1) = x0 + y;
         yc(2 * octant_size - i + 1) = y0 + x;
         
         % 6th octant
         xc(6 * octant_size + i) = x0 - y;
         yc(6 * octant_size + i) = y0 + x;
         
         % 7th octant
         xc(2 * octant_size + i) = x0 + y;
         yc(2 * octant_size + i) = y0 - x;
         
         % 8th octant
         xc(6 * octant_size - i + 1) = x0 - y;
         yc(6 * octant_size - i + 1) = y0 - x;
         
     end
     
points = [xc, yc];
end