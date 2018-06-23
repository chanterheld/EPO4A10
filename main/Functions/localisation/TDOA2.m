function [hpM,err] = TDOA2(y,x, pulse_fr,Fs)

err = 0;

y = squeeze(y);
SoS = 343;
maxD = 8;
primM = 5; %if changed change the find peaks hhats used;

%estimate channel response
for i = 1:5
    hhat(:,i) = ch3(y(:,i), x);
end

%
h_factor = 0.7;
bin_hhat = abs(hhat(:,primM)) >= (max(abs(hhat(:,primM)))*h_factor);
ind = 1;
peaks_found = 0;

%register every first peak and skip for .7 of per peak time
while ind <= length(bin_hhat)
    if bin_hhat(ind)
        peaks_found = peaks_found + 1;
        peak_pos(peaks_found) = ind;
        ind = ind + ceil(0.7*(1/pulse_fr)*Fs);
    end
    ind = ind + 1;
end

%remove peaks that may match peaks outside recorded time
half_bin = ceil((maxD/SoS)*Fs);
if peak_pos(1) <= half_bin
    peak_pos = peak_pos(2:peaks_found);
    peaks_found = peaks_found - 1;
end

if peak_pos(peaks_found) >= size(bin_hhat) - half_bin
    peak_pos = peak_pos(1:peaks_found-1);
    peaks_found = peaks_found - 1;
end

%find respective cutoffs for other responses
h1_max = max(hhat(half_bin:end-half_bin,1));
h2_max = max(hhat(half_bin:end-half_bin,2));
h3_max = max(hhat(half_bin:end-half_bin,3));
h4_max = max(hhat(half_bin:end-half_bin,4));
hp = zeros(peaks_found,5);

%find matching peaks
for i = 1:peaks_found
    [ind] = findFpeaks(hhat(peak_pos(i)-half_bin:peak_pos(i)+half_bin,1), h_factor*h1_max);
    hp(i,1) = ind(1)- half_bin -1;
    [ind] = findFpeaks(hhat(peak_pos(i)-half_bin:peak_pos(i)+half_bin,2), h_factor*h2_max);
    hp(i,2) = ind(1)- half_bin -1;
    [ind] = findFpeaks(hhat(peak_pos(i)-half_bin:peak_pos(i)+half_bin,3), h_factor*h3_max);
    hp(i,3) = ind(1)- half_bin -1;
    [ind] = findFpeaks(hhat(peak_pos(i)-half_bin:peak_pos(i)+half_bin,4), h_factor*h4_max);
    hp(i,4) = ind(1)- half_bin -1;
end

%debug plots
% for i = 1:5
%     subplot(2,3,i)
%     hold on
%     plot([peak_pos(:) + hp(:,i)],h_factor*max(hhat(half_bin:end-half_bin,i)),'*r')
%     hold off
% end

%cutoff sort and remove largest deviation peaks
hp = sort(hp);
try
hp = mean(hp(1:end-1,:),1);
catch
   err = 1; 
end
if size(hp, 1) < 1
    err = 1;
    return;
end

%convert to meters
hpM = hp*(SoS/Fs);
hpM = hpM(:);
end
