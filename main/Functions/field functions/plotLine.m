%https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm

function [points] = plotLine(P0, P1)
    x0 = P0(1);
    y0 = P0(2);
    x1 = P1(1);
    y1 = P1(2);
    
  if abs(y1 - y0) < abs(x1 - x0)
    if x0 > x1
      points = plotLineLow(x1, y1, x0, y0);
    else
      points = plotLineLow(x0, y0, x1, y1);
    end
  else
    if y0 > y1
      points = plotLineHigh(x1, y1, x0, y0);
    else
      points = plotLineHigh(x0, y0, x1, y1);
    end
  end
 end

function [points] = plotLineHigh(x0,y0, x1,y1)
  dx = x1 - x0;
  dy = y1 - y0;
  xi = 1;
  if dx < 0
    xi = -1;
    dx = -dx;
  end
  D = 2*dx - dy;
  x = x0;

  points = zeros(y1-y0+1,2);
  for y = y0:y1
   % field(x,y) = 1;
    points(y-y0+1,:) = [x,y]; 
    if D > 0
       x = x + xi;
       D = D - 2*dy;
    end
    D = D + 2*dx;
  end
end

function [points] = plotLineLow(x0,y0, x1,y1)
  dx = x1 - x0;
  dy = y1 - y0;
  yi = 1;
  if dy < 0
    yi = -1;
    dy = -dy;
  end
  D = 2*dy - dx;
  y = y0;

  points = zeros(x1-x0+1,2);
  for x = x0:x1;
   % field(x,y) = 1;
   points(x-x0+1,:) = [x,y]; 
    if D > 0
       y = y + yi;
       D = D - 2*dx;
    end
    D = D + 2*dy;
  end
end
    