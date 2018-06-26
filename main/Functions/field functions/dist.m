function [y] = dist(xt, xr)
   y = sqrt((xr(1) - xt(1))^2 + (xr(2) - xt(2))^2);
end
