function [obstacle_map] = place_obstacles(obstacle_map, Ncor, points, labels)
    Ncor = max(2,Ncor);
    fig_act = 0;
    while true
        if fig_act
            answer = questdlg('Add another obstacle?','Yes','yes','No','no');
        else
            answer = 'yes';
        end
        if strcmp(answer,'yes')
            if ~fig_act
                figure;
                pointer = subplot(1,1,1);
                fig_act = 1;
            end
            
            plot_pnt_lbl(obstacle_map, points, labels, pointer);
            [xs,ys] = ginput(1);
            [xss,yss] = deal(xs,ys);
            for i = 1:Ncor-1
                [x,y] = ginput(1);
                [obstacle_map] = place_points(obstacle_map, plotLine(round([xs, ys]), round([x y])));
                plot_pnt_lbl(obstacle_map, points, labels, pointer);
                [xs,ys] = deal(x,y);
            end
            [obstacle_map] = place_points(obstacle_map, plotLine(round([xs, ys]), round([xss yss])));
            plot_pnt_lbl(obstacle_map, points, labels, pointer);
            [xm,ym] = ginput(1);
            [obstacle_map] = fillout(obstacle_map,round(round([xm,ym])));            
            plot_pnt_lbl(obstacle_map, points, labels, pointer);
        else
            break;
        end
    end
end

function plot_pnt_lbl(map, points, labels, pointer)
imagesc(rot90(flipud(map),-1),'parent',pointer);
set(gca,'YDir','normal');
colormap(flipud(gray));            
hold on
plot([points(:,1)],[points(:,2)],'o')
text([points(:,1)],[points(:,2)],labels, 'VerticalAlignment','bottom','HorizontalAlignment','right')
hold off
end

