function [POS1, POS2,val1, val2] = get_car_position_test(test_data,audio,ratio,mic_locations)%(y_data,ref,pulse_rate)
compareV = [];
LOCVT = []; 
failed_cnt = 0;

    [y_data] = test_data;
    
    [D,err] = TDOA2(y_data,audio.ref,audio.pulse_rate,audio.Fs);
    if err
       failed_cnt = failed_cnt + 1;
       disp('TDOA ERROR')
    end
    

    [low,~] = min(mic_locations,[],1);
    [high,~] = max(mic_locations,[],1);    
    [POS1,val1] = lst_sq_pos3D(D*ratio, mic_locations, 8*ratio, .26*ratio, .02*ratio, [low(1), high(1)], [low(2), high(2)]);


   
    for i = 1:5
        Dt = D;
        Dt(i) = [];

        MICt = mic_locations;
        MICt(i,:) = [];

        [POS,val] = lst_sq_pos3D(Dt*ratio, MICt, 8*ratio, .26*ratio, .02*ratio, [low(1), high(1)], [low(2), high(2)]);
        
        LOCVT = [LOCVT;POS];
        compareV = [compareV;val];
    end
    
    [val2,ind] = min(compareV);
    POS2 = LOCVT(ind,:);

    
    
    
    
%     compareV = [compareV;x, y, score];
%     if score > 80
%         for i = 1:5
%             [x, y, score] = location_estimation(R*ratio,xlow,xhigh,ylow,yhigh,46*5,46*5,mic_locations,i);
%             compareV = [compareV;x, y, score];
%         end
%     end
%     
%     [score, ind] = min(compareV(:,3));
%     if score > 500
%         failed_cnt = failed_cnt + 1;
%         failed = 1;
%     else
%         failed = 0;
%     end     



% if isempty(score)
%     score = 1000;
% end
end

