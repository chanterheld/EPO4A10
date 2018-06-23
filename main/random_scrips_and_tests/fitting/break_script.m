lear all; close all; clc;
load('brake_matrix.mat');
[br_poly_obj] = poly23fit(T, Vvec, brake_tabl)
br_poly = coeffvalues(br_poly_obj);
save('br_poly1','br_poly','br_poly_obj');

plot(br_poly_obj)