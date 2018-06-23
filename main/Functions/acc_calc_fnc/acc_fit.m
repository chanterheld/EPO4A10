function [acc_cell] = acc_fit(m, a, b, c, Fmax, Vmax, Vsteps, Fsteps, progress)
progress.Message = 'Starting extrapolation simulation';
progress.Indeterminate = 'on';

acc_mod = 'KITTdyn_acc';

Vmargin = 5/3.6;
Fmargin = 10;

maxtime = .2;
maxstr = num2str(maxtime);

stepW = Fmax/(Fsteps-1);
addsteps = ceil(Fmargin/stepW);
addlength = stepW*addsteps;
Fsteps = Fsteps+addsteps;
T = linspace(0, Fmax+addlength, Fsteps);

T = [-fliplr(T(2:end)), T];
Fsteps = 2*Fsteps-1;
acc_cell{1,1} = T;
acc_cell{2,1} = Fsteps;

stepW = (Vmax/(Vsteps-1));
addsteps = ceil(Vmargin/stepW);
addlength = stepW*addsteps;
Vsteps = Vsteps+addsteps;
Vvec = linspace(0,Vmax + addlength,Vsteps);

Vvec = [-fliplr(Vvec(2:end)), Vvec];
Vsteps = 2*Vsteps-1;
acc_cell{1,2} = Vvec;
acc_cell{2,2} = Vsteps;

% loopvec(1,:) = kron(Vvec,ones(1,Fsteps));
% loopvec(2,:) = repmat(T,1,Vsteps);
% LL = size(loopvec,2);

loopvec2(1,:) = kron(acc_cell{1,2},ones(1,acc_cell{2,1}));
loopvec2(2,:) = repmat(acc_cell{1,1},1,acc_cell{2,2});
LL2 = size(loopvec2,2);

progress.Message = 'Creating simulink input';
progress.Indeterminate = 'off';
progress.Value = 0;

for i = 1:LL2
    in(i) = Simulink.SimulationInput(acc_mod);
    
    in(i) = in(i).setVariable('v0', loopvec2(1,i));
    in(i) = in(i).setVariable('F_ext', loopvec2(2,i));
    
    in(i) = in(i).setModelParameter('StopTime',maxstr);
    in(i) = in(i).setVariable('m', m);
    in(i) = in(i).setVariable('a', a);
    in(i) = in(i).setVariable('b', b);
    in(i) = in(i).setVariable('c', c);
    progress.Value = i/LL2;
end

progress.Message = 'Simulating';
progress.Indeterminate = 'on';

out = parsim(in, 'ShowSimulationManager', 'on' , 'ShowProgress' , 'off', 'UseFastRestart', 'on');

A = zeros(1,LL2);
B = zeros(1,LL2);
G = zeros(1,LL2);
L = zeros(1,LL2);
timeV = (linspace(0,maxtime,maxtime*1000+1))';

progress.Message = 'Fitting and converting output data';
progress.Indeterminate = 'off';
progress.Value = 0;

for k = 1:LL2
  YV = out(k).speed.data - loopvec2(1,k);
  fitX = fit(timeV, out(k).distance.data, 'poly2', 'Lower', [-Inf,-Inf,0], 'Upper', [Inf,Inf,0]);
  fitV = fit(timeV, YV, 'poly2', 'Lower', [-Inf,-Inf,0], 'Upper', [Inf,Inf,0]);
  
  coefX = coeffvalues(fitX);
  coefV = coeffvalues(fitV);
  A(k) = coefX(1);
  B(k) = coefX(2);
  G(k) = coefV(1);
  L(k) = coefV(2);
  
  progress.Value = k/LL2;
end

acc_cell{1,3} = vec2mat(A,acc_cell{2,1}); %A_tab
acc_cell{2,3} = vec2mat(B,acc_cell{2,1}); %B_tab
acc_cell{1,4} = vec2mat(G,acc_cell{2,1}); %G_tab
acc_cell{2,4} = vec2mat(L,acc_cell{2,1}); %L_tab

end
