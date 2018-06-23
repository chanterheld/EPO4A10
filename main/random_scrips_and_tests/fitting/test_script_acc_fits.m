clear all; close all; clc

m=13;
a = 5.85;
b = 14.1;
c = 0.5;
Fmax = 60;
Vmax = 20/3.6;
Vsteps = 20;
Fsteps = 15;
maxtime = .2;
maxstr = num2str(maxtime);



Vmargin = 5/3.6;
Fmargin = 10;

stepW = (Vmax/(Vsteps-1));
addsteps = ceil(Vmargin/stepW);
addlength = stepW*addsteps;
Vsteps = Vsteps+addsteps;
Vvec = linspace(0,Vmax + addlength,Vsteps);
Vvec = [-fliplr(Vvec(2:end)), Vvec];
Vsteps = 2*Vsteps-1;

stepW = Fmax/(Fsteps-1);
addsteps = ceil(Fmargin/stepW);
addlength = stepW*addsteps;
Fsteps = Fsteps+addsteps;
T = linspace(0, Fmax+addlength, Fsteps);
T = [-fliplr(T(2:end)), T];
Fsteps = 2*Fsteps-1;

loopvec(1,:) = kron(Vvec,ones(1,Fsteps));
loopvec(2,:) = repmat(T,1,Vsteps);

numSims = size(loopvec,2);

resX = zeros(numSims,maxtime*1000+1);
resV = zeros(numSims,maxtime*1000+1);

mdl = 'KITTdyn_acc';

parfor i = 1:numSims
 in(i) = Simulink.SimulationInput(mdl);
 in(i) = in(i).setVariable('v0', loopvec(1,i));
 in(i) = in(i).setVariable('F_ext', loopvec(2,i));
end
 in(:) = in(:).setModelParameter('StopTime',maxstr);
 in(:) = in(:).setVariable('m', m);
 in(:) = in(:).setVariable('a', a);
 in(:) = in(:).setVariable('b', b);
 in(:) = in(:).setVariable('c', c);
%%
out = parsim(in, 'ShowSimulationManager', 'on');

parfor k = 1:numSims
     resX(k,:) = out(k).distance.data;
     resV(k,:) = out(k).speed.data - loopvec(1,k);
end
timeV = (linspace(0,maxtime,maxtime*1000+1))';

save('temp_acc_save', 'resX','resV', 'timeV' ,'T','Vvec', 'Fsteps', 'out')

%%

clear all; close all; clc;
load('temp_acc_save');

LL = length(T)*length(Vvec);
A  = zeros(1,LL);
B  = zeros(1,LL);
G  = zeros(1,LL);
L  = zeros(1,LL);

parfor k = 1:LL
  fitX = fit(timeV,resX(k,:)','poly2','Lower',[-Inf,-Inf,0],'Upper',[Inf,Inf,0]);
  fitV = fit(timeV,resV(k,:)','poly2','Lower',[-Inf,-Inf,0],'Upper',[Inf,Inf,0]);
  coefX = coeffvalues(fitX);
  coefV = coeffvalues(fitV);
  A(k) = coefX(1);
  B(k) = coefX(2);
  G(k) = coefV(1);
  L(k) = coefV(2);
end

A_tab = vec2mat(A,Fsteps);
B_tab = vec2mat(B,Fsteps);
G_tab = vec2mat(G,Fsteps);
L_tab = vec2mat(L,Fsteps);

save('temp_acc_save2','A_tab', 'B_tab','G_tab','L_tab','T','Vvec');

%%
clear all; close all; clc;
load('temp_acc_save2');

ft = fittype( 'loess' );
opts = fitoptions( 'Method', 'LowessFit' );
opts.Normalize = 'on';

%A
[xData, yData, zData] = prepareSurfaceData( T, Vvec, A_tab );
opts.Span = 0.05;
Afit = fit([xData, yData], zData, ft, opts);

%B
[xData, yData, zData] = prepareSurfaceData( T, Vvec, B_tab );
opts.Span = 0.05;
Bfit = fit( [xData, yData], zData, ft, opts );

%G
[xData, yData, zData] = prepareSurfaceData( T, Vvec, G_tab );
opts.Span = 0.01;
Gfit = fit([xData, yData], zData, ft, opts);

%L
[xData, yData, zData] = prepareSurfaceData( T, Vvec, L_tab );
opts.Span = 0.05;
Lfit = fit( [xData, yData], zData, ft, opts );

save('temp_acc_save3','Afit','Bfit','Gfit','Lfit')