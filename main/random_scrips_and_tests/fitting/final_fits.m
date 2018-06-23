%fits
%A
[xData, yData, zData] = prepareSurfaceData( T, Vvec, A_tab );
% Set up fittype and options.
ft = fittype( 'loess' );
opts = fitoptions( 'Method', 'LowessFit' );
opts.Normalize = 'on';
opts.Span = 0.05;
% Fit model to data.
[fitresult, gof] = fit( [xData, yData], zData, ft, opts );

%Plot
figure( 'Name', 'untitled fit 1' );
h = plot( fitresult, [xData, yData], zData );
legend( h, 'untitled fit 1', 'A_tab vs. T, Vvec', 'Location', 'NorthEast' );
% Label axes
xlabel T
ylabel Vvec
zlabel A_tab
grid on
view( 87.3, 27.4 );


%B
[xData, yData, zData] = prepareSurfaceData( T, Vvec, B_tab );
% Set up fittype and options.
ft = fittype( 'loess' );
opts = fitoptions( 'Method', 'LowessFit' );
opts.Normalize = 'on';
opts.Span = 0.05;
% Fit model to data.
[fitresult, gof] = fit( [xData, yData], zData, ft, opts );


% Plot fit with data.
figure( 'Name', 'untitled fit 1' );
h = plot( fitresult, [xData, yData], zData );
legend( h, 'untitled fit 1', 'B_tab vs. T, Vvec', 'Location', 'NorthEast' );
% Label axes
xlabel T
ylabel Vvec
zlabel B_tab
grid on
view( 92.4, -2.6 );


%G
[xData, yData, zData] = prepareSurfaceData( T, Vvec, G_tab );
% Set up fittype and options.
ft = fittype( 'loess' );
opts = fitoptions( 'Method', 'LowessFit' );
opts.Normalize = 'on';
opts.Span = 0.01;
% Fit model to data.
[fitresult, gof] = fit( [xData, yData], zData, ft, opts );

% Plot fit with data.
figure( 'Name', 'untitled fit 1' );
h = plot( fitresult, [xData, yData], zData );
legend( h, 'untitled fit 1', 'G_tab vs. T, Vvec', 'Location', 'NorthEast' );
% Label axes
xlabel T
ylabel Vvec
zlabel G_tab
grid on
view( -225.7, 43.4 );

%L
[xData, yData, zData] = prepareSurfaceData( T, Vvec, L_tab );
% Set up fittype and options.
ft = fittype( 'loess' );
opts = fitoptions( 'Method', 'LowessFit' );
opts.Normalize = 'on';
opts.Span = 0.05;
% Fit model to data.
[fitresult, gof] = fit( [xData, yData], zData, ft, opts );

% Plot fit with data.
figure( 'Name', 'untitled fit 1' );
h = plot( fitresult, [xData, yData], zData );
legend( h, 'untitled fit 1', 'L_tab vs. T, Vvec', 'Location', 'NorthEast');
% Label axes
xlabel T
ylabel Vvec
zlabel L_tab
grid on
view( 88.3, 24.2 );



