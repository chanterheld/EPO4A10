function [field,MIC] = create_map(MIC,scaling,order)
order = [order;order(1)];
N = size(MIC,1);

mic_cirle = round(.30*scaling);
margin = 5+mic_cirle;
MIC = round(MIC.*scaling);
MICZ = MIC(:,3);

deltaMIN = min(MIC,[],1);
MIC = [MIC(:,1)-deltaMIN(1),MIC(:,2)-deltaMIN(2)]+margin;
MIC = [MIC,MICZ];
deltaMAX = max(MIC,[],1);

field = zeros(deltaMAX(1)+margin,deltaMAX(2)+margin);


for k = 1:N
    i = order(k);
    diskpoints = draw_disk(MIC(i,1:2), mic_cirle);
    linepoints = plotLine(MIC(i,1:2), MIC(order(k+1),1:2));
    [field] = place_points(field, [diskpoints;linepoints]);
end

field = fillout(field,[1,1]);
imagesc(field)
colormap(flipud(gray));

end

