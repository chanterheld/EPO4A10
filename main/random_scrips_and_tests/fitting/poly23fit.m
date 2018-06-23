function [fitresult] = poly23fit(X, Y, Z_tab)
[xData, yData, zData] = prepareSurfaceData(X, Y, Z_tab);
ft = fittype('poly23');
[fitresult, ~] = fit( [xData, yData], zData, ft );


