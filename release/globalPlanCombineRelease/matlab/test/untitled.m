% 测试例
close all;
clear all;
r = 5;           % 圆的半径
A = [0, 0];      % 点A的坐标 [x, y]
B = [5, 0];      % 点B的坐标 [x, y]
C = [0, 12];     % 点C的坐标 [x, y]

[distance, O, tangent_point_AB, tangent_point_BC] = distance_to_tangent_point(r, A, B, C);
% 调用函数画图展示点A、点B、点C以及与AB和BC相切的圆
figure;
plot_circle(A, B, C, O, r,tangent_point_AB, tangent_point_BC)
