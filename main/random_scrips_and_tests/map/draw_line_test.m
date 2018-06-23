clear all; close all; clc

field = zeros(100);

Dest = [   1 1;
        1 25;
        1 50;
        1 75;
        1 100;
        25 100;
        50 100;
        75 100;
        100 100;
        100 75;
        100 50;
        100 25;
        100 1;
        75 1;
        50 1;
        25 1];
        
for i = 1:size(Dest,1)

[points] = plotLine([50 50], Dest(i,:));
field = place_points(field, points);
imagesc(field)
pause(1)

end