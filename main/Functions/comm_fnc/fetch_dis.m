function [dis, err] = fetch_dis()
err = 1;
n = 0;
while err && n<3
    status = 0;
    status = EPOCommunications('transmit', 'Sd');
    if isa(status,'char')
        string = strsplit(status,{'R','L', '\n'});
        distanceL = str2num(string{1,2});
        distanceR= str2num(string{1,4});
        err = 0;
    else
        n = n + 1;
    end
end

if err
    dis = 4;
    return
end

if distanceL == 0
    distanceL = 999;
end

if distanceR == 0
    distanceR = 999;
end

if abs(distanceL - distanceR) < 40
    dis = (distanceL + distanceR)/200;
else
    dis = min(distanceL, distanceR)/100;
end

end

