function [corners,mid] = FoF_poly(pos,heading,distance,ratio)
carL = .4*ratio;
distance = distance*ratio;
sensor_spacing = .3 *ratio;
sensor_angle = 10; 
close_cor = [   pos + extr_pos(heading-90,sensor_spacing/2);
                pos + extr_pos(heading+90,sensor_spacing/2)];
            
far_cor = [ close_cor(1,:) + extr_pos(heading-sensor_angle,distance);
            close_cor(2,:) + extr_pos(heading+sensor_angle,distance)];
            
back_cor = [    close_cor(1,:) + extr_pos(heading+180,carL);
                close_cor(2,:) + extr_pos(heading+180,carL)];
            
corners = round([back_cor;close_cor(2,:);flipud(far_cor);close_cor(1,:)]);

mid = round(pos + extr_pos(heading,distance/2));
end