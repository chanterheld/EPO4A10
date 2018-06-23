%clear all; close all; clc

m=13;
a = 5.85;
b = 14.1;
c = 0.5;
Fmax = 60;
Vmax = 20/3.6;
Vsteps = 20;
Fsteps = 15;

%[break_cell] = brake_fit(m, a, b, c, Fmax, Vmax, Vsteps, Fsteps);
[acc_cell] = acc_fit(m, a, b, c, Fmax, Vmax, Vsteps, Fsteps);
save('prefit','acc_cell', 'break_cell')
%%
% save('temp_brake_save', 'brake_tabl','time_tabl','T','Vvec')
% 
% %%
% clear all; close all; clc;
% load('temp_brake_save');

%function fitted using lowess setting at 10% span
%better to expand F beond Fmax to get better fit;

load('prefit','acc_cell', 'break_cell')

ft = fittype( 'loess' );
opts = fitoptions( 'Method', 'LowessFit' );
opts.Normalize = 'on';

%X
T = break_cell{1,1};
Vvec = break_cell{1,2};
brake_tabl = break_cell{1,3};

[xData, yData, zData] = prepareSurfaceData( T, Vvec, brake_tabl );
opts.Span = 0.1;
BR_Xfit = fit([xData, yData], zData1, ft, opts);

figure(1); plot(BR_Xfit, [xData, yData], zData);


%T
T = break_cell{1,1};
Vvec = break_cell{1,2};
time_tabl = break_cell{2,3};

[xData, yData, zData] = prepareSurfaceData( T, Vvec, time_tabl);
opts.Span = 0.05;
%BR_Tfit = fit([xData, yData], zData2, ft, opts);

figure(2); plot(BR_Tfit, [xData, yData], zData);

%A
T = acc_cell{1,1};
Vvec = acc_cell{1,2};
A_tab = acc_cell{1,3};

[xData, yData, zData] = prepareSurfaceData( T, Vvec, A_tab );
opts.Span = 0.05;
%Afit = fit([xData, yData], zData, ft, opts);

figure(3); plot(Afit, [xData, yData], zData);

%B
T = acc_cell{1,1};
Vvec = acc_cell{1,2};
B_tab = acc_cell{2,3};

[xData, yData, zData] = prepareSurfaceData(T, Vvec, B_tab);
opts.Span = 0.05;
%Bfit = fit( [xData, yData], zData, ft, opts );

figure(4); plot(Bfit, [xData, yData], zData);

%%

%G
T = acc_cell{1,1};
Vvec = acc_cell{1,2};
G_tab = acc_cell{1,4};

[xData, yData, zData] = prepareSurfaceData( T, Vvec, G_tab );
opts.Span = 0.01;
Gfit = fit([xData, yData], zData, ft, opts);

figure(5); plot(Gfit, [xData, yData], zData);

%L
T = acc_cell{1,1};
Vvec = acc_cell{1,2};
L_tab = acc_cell{2,4};

[xData, yData, zData] = prepareSurfaceData( T, Vvec, L_tab );
opts.Span = 0.05;
%Lfit = fit( [xData, yData], zData, ft, opts );

figure(6); plot(Lfit, [xData, yData], zData);

% save('break_fits', 'BR_Xfit', 'BR_Tfit')

%%


