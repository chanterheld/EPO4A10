function [TTS,TimeVec] = spd_pr_fit(m, a, b, c, Fmax, Trange)

acc_mod = 'KITTdyn_speed';
TSS = 0.1; %Time Step Size

TimeVec = (Trange(1):TSS:Trange(2));
TrqVec = linspace(0,Fmax,16);

 LL2 = size(TrqVec,2);


for i = 1:LL2
    in(i) = Simulink.SimulationInput(acc_mod);
    
    in(i) = in(i).setVariable('F_ext', TrqVec(i));
    
    in(i) = in(i).setModelParameter('StopTime',num2str(Trange(2)));
    in(i) = in(i).setVariable('m', m);
    in(i) = in(i).setVariable('a', a);
    in(i) = in(i).setVariable('b', b);
    in(i) = in(i).setVariable('c', c);
end

out = parsim(in, 'ShowSimulationManager', 'on' , 'ShowProgress' , 'off', 'UseFastRestart', 'on');

TTS = zeros(16,length(TimeVec));

for k = 1:LL2
    SimTVec = out(k).speed.time;
    SimVVec = out(k).speed.data;
  for l = 1:length(TimeVec)
      [~,ind] = min(abs(SimTVec - TimeVec(l)));
      TTS(k,l) = SimVVec(ind);
  end
end

end
