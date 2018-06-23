function [MID,rot_ang] = mid_point(Pos,R,alpha, dir)
    switch dir
        case 'L'
            MID = round(Pos + [-sind(alpha),cosd(alpha)].*R);
            rot_ang = alpha - 90;
        case 'R'
            MID = round(Pos + [sind(alpha),-cosd(alpha)].*R);
            rot_ang = alpha + 90;
    end
    if rot_ang<-180
        rot_ang = rot_ang + 360;
    elseif rot_ang > 180
        rot_ang = rot_ang - 360;
    end
end

