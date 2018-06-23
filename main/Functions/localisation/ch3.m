function h = ch3(y,x)

x = x(:);
y = y(:);
Nx = length(x);
Ny = length(y);
L = Ny - Nx + 1;

Y = fft(y);
X = fft([x; zeros(L-1,1)]);
X = X(:);
Y = Y(:);

H = Y./X;

X_max = max(X);
threshold = 0.1 * X_max;
ii = find(abs(X) > threshold);

G = zeros(Ny,1);
for n = 1:length(ii)
    G(ii(n)) = 1;
end

h = ifft(H.*G);
h = abs(h(1:L));
end


