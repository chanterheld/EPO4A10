clear all; close all; clc;

m = 13;
Fmax = 60;
Vmax = 20/3.6;
Vsteps = ceil(Vmax/.2);
Trange = [.3, 4];

%straight
a = 6.96;
b = 15.24;
c = 0.3804;

ft = fittype( 'loess' );
opts = fitoptions( 'Method', 'LowessFit' );
opts.Normalize = 'on';
            

[TTS_str,~] = spd_pr_fit(m, a, b, c, Fmax, Trange);

[brake_cell] = brake_fit_alt(m, a, b, c, Fmax, Vmax, Vsteps, 25);

T = brake_cell{1,1};
Vvec = brake_cell{1,2};
time_tabl = brake_cell{2,3};

[xData, yData, zData] = prepareSurfaceData( T, Vvec, time_tabl);
opts.Span = 0.05;
BR_Tfit_str = fit([xData, yData], zData, ft, opts);


%turn
a = 10.44;
b = 15.83;
c = 4.954;

[TTS_trn,TimeVec] = spd_pr_fit(m, a, b, c, Fmax, Trange);

[brake_cell] = brake_fit_alt(m, a, b, c, Fmax, Vmax, Vsteps, 25);

T = brake_cell{1,1};
Vvec = brake_cell{1,2};
time_tabl = brake_cell{2,3};

[xData, yData, zData] = prepareSurfaceData( T, Vvec, time_tabl);
opts.Span = 0.05;
BR_Tfit_trn = fit([xData, yData], zData, ft, opts);

save('drive_stop_fits', 'TTS_str', 'TTS_trn', 'BR_Tfit_str', 'BR_Tfit_trn', 'TimeVec', 'Fmax');
