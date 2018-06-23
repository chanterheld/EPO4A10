clear all;
close all
clc

EPOCommunications('open','\\.\COM3')

%%

nr_pulses = 6;
pulse_fr = 10;

f_c = 10000; %carier
f_b = 4000;
c_r = f_b/pulse_fr;

Trec = nr_pulses*(1/pulse_fr);
Fs= 48000;
nSamples = ceil(Trec*Fs);

firstchannel = 1;
lastchannel = 5;

deviceid = 0;
devicetype = 'asio';
y = zeros(nSamples, 5);
buf = zeros(nSamples, 5);

EPOCommunications('transmit', 'A0');   % switch off audio beacon
EPOCommunications('transmit', ['B' num2str(f_b,'%d')] );        % set the bit frequency
EPOCommunications('transmit', ['F' num2str(f_c,'%d')]);       % set the carrier frequency
EPOCommunications('transmit', ['R' num2str(c_r,'%d')]);        % set the repetition count
EPOCommunications('transmit', ['C0x' code]);  % set the audio code
EPOCommunications('transmit', 'A1');   % switch off audio beacon

y = pa_wavrecord(firstchannel, lastchannel, nSamples, Fs, deviceid, devicetype);

EPOCommunications('transmit', 'A0');   % switch off audio beacon

%%
%chop up y to one pulse and call it x
save('refsignal', 'x')



%%
clear all;
close all
clc
load('micx_hok', 'micx5');

nr_pulses = 6;
pulse_fr = 10;
hhat_cutoff = 0.7;
prim_mic = 5;
maxD = 8;

%EPOCommunications('transmit', 'A1');   % switch off audio beacon

[distances, hhat, peaks_mat] = get_distances(micx5,pulse_fr,nr_pulses, hhat_cutoff, prim_mic, maxD)

%EPOCommunications('transmit', 'A0');   % switch off audio beacon






%%

EPOCommunications('close')
