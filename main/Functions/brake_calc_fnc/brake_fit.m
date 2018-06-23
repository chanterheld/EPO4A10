function [break_cell] = brake_fit(m, a, b, c, Fmax, Vmax, Vsteps, Fsteps, progress)
progress.Message = 'Starting extrapolation simulation';
progress.Indeterminate = 'on';

brake_mod = 'KITTdyn_brake';

Vmargin = 5/3.6;
Fmargin = 10;

maxtime = .5;
maxstr = num2str(maxtime);

stepW = Fmax/(Fsteps-1);
addsteps = ceil(Fmargin/stepW);
addlength = stepW*addsteps;
Fsteps = Fsteps+addsteps;
T = linspace(0, Fmax+addlength, Fsteps);
break_cell{1,1} = T;
break_cell{2,1} = Fsteps;

stepW = (Vmax/(Vsteps-1));
addsteps = ceil(Vmargin/stepW);
addlength = stepW*addsteps;
Vsteps = Vsteps+addsteps;
Vvec = linspace(0,Vmax + addlength,Vsteps);
break_cell{1,2} = Vvec;
break_cell{2,2} = Vsteps;

% loopvec(1,:) = kron(Vvec,ones(1,Fsteps));
% loopvec(2,:) = repmat(T,1,Vsteps);
% LL = size(loopvec,2);

loopvec1(1,:) = kron(break_cell{1,2},ones(1,break_cell{2,1}));
loopvec1(2,:) = repmat(break_cell{1,1},1,break_cell{2,2});
LL1 = size(loopvec1,2);

progress.Message = 'Creating simulink input';
progress.Indeterminate = 'off';
progress.Value = 0;

for i = 1:LL1
    in(i) = Simulink.SimulationInput(brake_mod);
    in(i) = in(i).setVariable('v0', loopvec1(1,i));
    in(i) = in(i).setVariable('F_ext', -loopvec1(2,i));
    
    in(i) = in(i).setVariable('m', m);
    in(i) = in(i).setVariable('a', a);
    in(i) = in(i).setVariable('b', b);
    in(i) = in(i).setVariable('c', c);
    progress.Value = i/LL1;
end

progress.Message = 'Simulating';
progress.Indeterminate = 'on';

out = parsim(in, 'ShowSimulationManager', 'on' , 'ShowProgress' , 'off', 'UseFastRestart', 'on');

progress.Message = 'Converting output data';
progress.Indeterminate = 'off';
progress.Value = 0;

resF = zeros(1,LL1);
resT = zeros(1,LL1);
for k = 1:LL1
    resT(k) = out(k).distance.time(end);    
    resF(k) = out(k).distance.data(end);
    progress.Value = k/LL1;
end

break_cell{1,3} = vec2mat(resF,break_cell{2,1}); %Fsteps
break_cell{2,3} = vec2mat(resT,break_cell{2,1});
end

