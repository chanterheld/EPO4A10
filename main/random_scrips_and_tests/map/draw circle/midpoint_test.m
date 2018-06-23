clear all
close all


field = zeros(1000,1000);
%[field] = plotLine(field,500,500, 500, 1);
pt0.x = 25;
pt0.y = 25;
pt1.x = 800;
pt1.y = 600;
field = murphy_line_draw(field, pt0, pt1, 10);

field(25,25) = 3;
field(800,600) = 3;

% tic
% field = drawcircle(field,[500;200], 200);
% toc
% figure;imagesc(field)
% tic
%field = fillout(field,[1000,1]);
% toc
figure;imagesc(field)






  
