function [points] = arc_sel(angles,circle,dir)
angles = angles - 90;
for i = 1:2
if angles(i) <-180
    angles(i)  = angles(i)  + 360;
elseif angles(i)  > 180
    angles(i)  = angles(i)  - 360;
end
end
%angle conversion
angles = angles*-1;
angles(1) = angle_360conv(angles(1));
angles(2) = angle_360conv(angles(2));
L = size(circle,1);
ind = 1;


switch dir
    case 'L'
      Sind = ceil((L/360)*angles(1));
      if Sind > L
         Sind = L; 
      end
      Find = floor((L/360)*angles(2));
      if Find < 1
          Find = 1;
      end
      while true
          points(ind,:) = circle(Sind,:);
          if Sind == Find
              return
          elseif Sind == 1
              Sind = L;
          else
              Sind = Sind - 1;
          end
          ind = ind + 1;
      end
      
    case 'R'
      Sind = floor((L/360)*angles(1));
      if Sind < 1
          Sind = 1;
      end
      Find = ceil((L/360)*angles(2));
      if Find > L
          Find = L;
      end
      
      while true
          points(ind,:) = circle(Sind,:);
          if Sind == Find
              return
          elseif Sind == L
              Sind = 1;
          else
              Sind = Sind + 1;
          end
          ind = ind + 1;
      end
end

end

function angle = angle_360conv(angle)
    if angle < 0
       angle = angle + 360; 
    end         
end

