function[POS] = lst_sq_pos3D(D, MIC, maxL, Z, Step, xmargin, ymargin)
nmic = size(MIC,1);
A = zeros(nmic - 1,2);
b2 = zeros(nmic - 1,1);
b1s = zeros(nmic - 1,2);
k = sum(MIC.^2,2);

r = zeros(nmic);
for i = 1:nmic
    for j = 1:nmic
        r(i,j) = D(i)-D(j);
    end
end

for i = 2:nmic    
    A(i-1,:) = [MIC(i,1)-MIC(1,1), MIC(i,2)-MIC(1,2)];    
    b2(i-1) = k(i)-k(1) - 2*Z*(MIC(i,3) - MIC(1,3));
    b1s(i-1,:) = [D(i), D(1)];
end
A_inv = pinv(A);

E = [D;0];
d_start = min(E(E<=0))*-1;
d = [d_start:Step:d_start+maxL];
Ld = length(d);
ind = 1;
LOCV = zeros(Ld,2);
ScoreVEC = zeros(Ld,1);
for kI = 1:Ld
    dl = d(kI);    
    b = diff((b1s + dl).^2 ,1,2) +b2;
    LOCL = (A_inv*b/2)';
%    score = score_cnt(LOCL, r,D, MIC,nmic,Z);
    if ~(LOCL(1) < xmargin(1) || LOCL(1) > xmargin(2) || LOCL(2) < ymargin(1) || LOCL(2) > ymargin(2))
        LOCV(ind,:) = LOCL;
        ScoreVEC(ind) = score_cnt(LOCL, r, MIC,Z,nmic);
        ind = ind + 1;
    end
end
ScoreVEC = ScoreVEC(1:ind-1);
[val, ind] = min(SocreVEC);
POS = [LOCV(ind,:),val];
end

function score = score_cnt(LOCL, r, MIC,Z,nmic)
DP = sqrt(sum((MIC - [LOCL,Z]).^2,2));

RP = zeros(nmic);
for i = 1:nmic
    for j = 1:nmic
        RP(i,j) = DP(i)-DP(j);
    end
end
scoreM = RP - r;
score = sum(sum(abs(scoreM)));
end