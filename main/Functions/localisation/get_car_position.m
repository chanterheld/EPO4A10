function [POS,score] = get_car_position(audio,ratio,mic_locations)
compareV = zeros(5,3);
failed_cnt = 0;
failed = 1;
while (failed && (failed_cnt < 6))
    [y_data] = record(audio.Fs,audio.nsamples);
    
    [D,err] = TDOA2(y_data,audio.ref,audio.pulse_rate,audio.Fs);
    if err
        failed_cnt = failed_cnt + 1;
        disp('TDOA ERROR')
        continue;
    end
    
    [low,~] = min(mic_locations,[],1);
    [high,~] = max(mic_locations,[],1);
    
    for i = 1:5
        Dt = D;
        Dt(i) = [];
        MICt = mic_locations;
        MICt(i,:) = [];
        compareV(i,:) = lst_sq_pos3D(Dt*ratio, MICt, 8*ratio, .26*ratio, .02*ratio, [low(1), high(1)], [low(2), high(2)]);
    end
    
    [score, ind] = min(compareV(:,3));
    if score > 25
        failed_cnt = failed_cnt + 1;
        failed = 1;
    else
        failed = 0;
    end  
end
POS = compareV(ind,1:2);
if isempty(score)
    score = 1000;
end
end

