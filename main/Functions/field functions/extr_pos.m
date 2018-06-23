function [delta] = extr_pos(alpha,dis)
delta = [cosd(alpha),sind(alpha)].*dis;
end

