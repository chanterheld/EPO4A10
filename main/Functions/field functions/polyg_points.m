function [points] = polyg_points(corners,mid)
margin = 5;
Nc = size(corners,1);
deltaMIN = min(corners,[],1);
corners = [corners(:,1)-deltaMIN(1),corners(:,2)-deltaMIN(2)]+margin;
mid = [mid(1)-deltaMIN(1),mid(2)-deltaMIN(2)]+margin;
deltaMAX = max(corners,[],1);
tempfield = zeros(deltaMAX(1)+margin,deltaMAX(2)+margin);


tmp_corners = [corners;corners(1,:)];
for i = 1:Nc
    [Lpoints] = plotLine(tmp_corners(i,:), tmp_corners(i+1,:));
    [tempfield] = place_points(tempfield, Lpoints);
end

[tempfield] = fillout(tempfield,mid);        
[x, y] = find(tempfield == 1);
points = [x + deltaMIN(1), y + deltaMIN(2)]-margin;

end

