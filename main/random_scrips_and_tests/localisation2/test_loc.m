clear all; close all; clc
 load('meting.mat','f_b', 'f_c', 'c_r', 'code');
 load('ref5','ref');

audio.Fs = 48000;
audio.f_b = f_b;
audio.f_c = f_c;
audio.c_r = c_r;
audio.code = code;
audio.ref = ref;
audio.pulse_rate = audio.f_b/audio.c_r;
audio.peaks_measured =6;
audio.nsamples = (1/audio.pulse_rate)*(audio.peaks_measured + 1)*audio.Fs;
clear x;

  load('meting.mat','f_b','c_r','RXXr')
load('ref5','ref');
xref = ref;


LOC = zeros(9,2);
mic_locations = [460,0,50; 0,0,50; 0,460,50; 460,460,50; 230,460,80];


for i = 1:9
 meting = RXXr(i,:,:);
 [POS1(i,:),POS2(i,:),score1(i,:),score2(i,:)] = get_car_position_test(meting,audio,100,mic_locations);
end


REF = [mic_locations(:,1:2);150 150; 200 360; 400 200; 230 230];

DLOC1xy =  (POS1 - [REF]);

DLOC1 = sqrt(sum(DLOC1xy.^2,2));

DLOC2xy =  (POS2 - [REF]);

DLOC2 = sqrt(sum(DLOC2xy.^2,2));





