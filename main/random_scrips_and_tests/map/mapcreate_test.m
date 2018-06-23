clear all
%close all
clc

% MIC = [ 4.6,    0,      .5;
%         0,      0,      .5;
%         0,      4.6,    .5;
%         4.6,    4.6,    .5;
%         2.3,    4.6,    .8];   
% 
% N = size(points,1);
% D = zeros(N,N);
%         
%  for i = 1:N
%      for k = 1:N
%          if i == k
%             continue 
%          end
%          D(i,k) = dist(points(i,:), points(k,:));
%      end 
%  end
load('roomK', 'MIC', 'order', 'prim');

[map,MIC] = create_map(MIC,100,order);

figure;imagesc(map)
colormap(flipud(gray));
colorbar

figure;plot(MIC(:,1),MIC(:,2),'*r')
% plot(MIC(:,1),MIC(:,2),'*r');
% hold on;
% plot(points(:,1),points(:,2),'o');


function [y] = dist(P1, P2);
   y = sqrt((P2(1) - P1(1))^2 + (P2(2) - P1(2))^2);
end
